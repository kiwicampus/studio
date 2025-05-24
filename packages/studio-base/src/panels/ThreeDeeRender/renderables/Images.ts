// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import { t } from "i18next";
import { assert } from "ts-essentials";

import { MultiMap, filterMap } from "@foxglove/den/collection";
import { PinholeCameraModel } from "@foxglove/den/image";
import Logger from "@foxglove/log";
import { toNanoSec } from "@foxglove/rostime";
import { CameraCalibration, CompressedImage, RawImage } from "@foxglove/schemas";
import { SettingsTreeAction, SettingsTreeFields } from "@foxglove/studio";
import { ALL_SUPPORTED_IMAGE_SCHEMAS } from "@foxglove/studio-base/panels/ThreeDeeRender/renderables/ImageMode/ImageMode";

import {
  IMAGE_RENDERABLE_DEFAULT_SETTINGS,
  ImageRenderable,
  ImageUserData,
} from "./Images/ImageRenderable";
import { ALL_CAMERA_INFO_SCHEMAS, AnyImage, CompressedVideo } from "./Images/ImageTypes";
import {
  normalizeCompressedImage,
  normalizeCompressedVideo,
  normalizeRawImage,
  normalizeRosCompressedImage,
  normalizeRosImage,
} from "./Images/imageNormalizers";
import { getTopicMatchPrefix, sortPrefixMatchesToFront } from "./Images/topicPrefixMatching";
import { cameraInfosEqual, normalizeCameraInfo } from "./projections";
import type { AnyRendererSubscription, IRenderer } from "../IRenderer";
import { PartialMessageEvent, SceneExtension, onlyLastByTopicMessage } from "../SceneExtension";
import { SettingsTreeEntry } from "../SettingsManager";
import {
  CAMERA_CALIBRATION_DATATYPES,
  COMPRESSED_IMAGE_DATATYPES,
  COMPRESSED_VIDEO_DATATYPES,
  RAW_IMAGE_DATATYPES,
} from "../foxglove";
import {
  CameraInfo,
  Image as RosImage,
  CompressedImage as RosCompressedImage,
  IMAGE_DATATYPES as ROS_IMAGE_DATATYPES,
  COMPRESSED_IMAGE_DATATYPES as ROS_COMPRESSED_IMAGE_DATATYPES,
  CAMERA_INFO_DATATYPES,
} from "../ros";
import { BaseSettings, PRECISION_DISTANCE } from "../settings";
import { topicIsConvertibleToSchema } from "../topicIsConvertibleToSchema";
import { makePose } from "../transforms";

const log = Logger.getLogger(__filename);
void log;

export type LayerSettingsImage = BaseSettings & {
  cameraInfoTopic: string | undefined;
  distance: number;
  planarProjectionFactor: number;
  color: string;
  segmentationMaskTopic: string | undefined;
  segmentationMaskOpacity: number;
};

const DEFAULT_BITMAP_WIDTH = 512;
const NO_CAMERA_INFO_ERR = "NoCameraInfo";
const CAMERA_MODEL = "CameraModel";

export class Images extends SceneExtension<ImageRenderable> {
  public static extensionId = "foxglove.Images";
  /* All known camera info topics */
  #cameraInfoTopics = new Set<string>();

  /**
   * A bi-directional mapping between cameraInfo topics and image topics. This
   * is used for retrieving an image renderable, which is indexed by image
   * topic, when receiving a camera info message.
   */
  #cameraInfoToImageTopics = new MultiMap<string, string>();

  /**
   * Map of camera info topic name -> normalized CameraInfo message
   *
   * This stores the last camera info message on each topic so it can be applied when rendering the image
   */
  #cameraInfoByTopic = new Map<string, CameraInfo>();

  /**
   * A bi-directional mapping between segmentation mask topics and image topics.
   * Used to find which ImageRenderables need updating when a mask message arrives.
   */
  #segmentationMaskToImageTopics = new MultiMap<string, string>();

  protected supportedImageSchemas = ALL_SUPPORTED_IMAGE_SCHEMAS;

  public constructor(renderer: IRenderer, name: string = Images.extensionId) {
    super(name, renderer);
    this.renderer.on("topicsChanged", this.#handleTopicsChanged);
    this.#handleTopicsChanged();
  }

  public override dispose(): void {
    this.renderer.off("topicsChanged", this.#handleTopicsChanged);
    super.dispose();
  }

  public override getSubscriptions(): readonly AnyRendererSubscription[] {
    const subscriptions: AnyRendererSubscription[] = [
      {
        type: "schema",
        schemaNames: ALL_CAMERA_INFO_SCHEMAS,
        subscription: {
          handler: this.#handleCameraInfo,
          shouldSubscribe: this.#cameraInfoShouldSubscribe,
        },
      },
    ];

    // Helper to create image subscriptions
    const createImageSubscription = (
      schemaNames: readonly string[],
      normalizer: (message: AnyImage) => AnyImage,
    ): AnyRendererSubscription => ({
      type: "schema",
      schemaNames,
      subscription: {
        handler: (messageEvent: PartialMessageEvent<AnyImage>) => {
          // Normalize and route the message
          this.#handleAnyImageMessage(messageEvent, normalizer(messageEvent.message));
        },
        shouldSubscribe: this.#imageShouldSubscribe, // Unified subscription logic
        filterQueue: onlyLastByTopicMessage,
      },
    });

    subscriptions.push(
      createImageSubscription(ROS_IMAGE_DATATYPES, normalizeRosImage),
      createImageSubscription(ROS_COMPRESSED_IMAGE_DATATYPES, normalizeRosCompressedImage),
      createImageSubscription(RAW_IMAGE_DATATYPES, normalizeRawImage),
      createImageSubscription(COMPRESSED_IMAGE_DATATYPES, normalizeCompressedImage),
      createImageSubscription(COMPRESSED_VIDEO_DATATYPES, normalizeCompressedVideo),
    );

    return subscriptions;
  }

  /**
   * Update cameraInfoTopics cache with latest set of camera info messages
   */
  #handleTopicsChanged = () => {
    this.#cameraInfoTopics = new Set();
    for (const topic of this.renderer.topics ?? []) {
      if (
        topicIsConvertibleToSchema(topic, CAMERA_INFO_DATATYPES) ||
        topicIsConvertibleToSchema(topic, CAMERA_CALIBRATION_DATATYPES)
      ) {
        this.#cameraInfoTopics.add(topic.name);
      }
    }
  };

  public override settingsNodes(): SettingsTreeEntry[] {
    const configTopics = this.renderer.config.topics;
    const handler = this.handleSettingsAction;
    const entries: SettingsTreeEntry[] = [];
    for (const topic of this.renderer.topics ?? []) {
      if (!topicIsConvertibleToSchema(topic, this.supportedImageSchemas)) {
        continue;
      }
      const imageTopic = topic.name;
      const config = (configTopics[imageTopic] ?? {}) as Partial<LayerSettingsImage>;

      // Build a list of all matching CameraInfo topics
      const cameraInfoOptions = Array.from(this.#cameraInfoTopics, (topicName) => ({
        label: topicName,
        value: topicName,
      }));
      cameraInfoOptions.sort();
      sortPrefixMatchesToFront(cameraInfoOptions, imageTopic, (option) => option.value);

      const fields: SettingsTreeFields = {
        cameraInfoTopic: {
          label: t("threeDee:cameraInfo"),
          input: "select",
          options: cameraInfoOptions,
          value: config.cameraInfoTopic,
        },
        distance: {
          label: t("threeDee:distance"),
          input: "number",
          placeholder: String(IMAGE_RENDERABLE_DEFAULT_SETTINGS.distance),
          step: 0.1,
          precision: PRECISION_DISTANCE,
          value: config.distance,
        },
        planarProjectionFactor: {
          label: t("threeDee:planarProjectionFactor"),
          input: "number",
          placeholder: String(IMAGE_RENDERABLE_DEFAULT_SETTINGS.planarProjectionFactor),
          min: 0,
          max: 1,
          step: 0.1,
          precision: 2,
          value: config.planarProjectionFactor,
        },
        color: { label: t("threeDee:color"), input: "rgba", value: config.color },
        segmentationMaskTopic: {
          label: t("threeDee:segmentationMaskTopic", "Segmentation Mask Topic"),
          input: "select",
          options: cameraInfoOptions, // For now, use the same options as cameraInfo
          value: config.segmentationMaskTopic,
        },
        segmentationMaskOpacity: {
          label: t("threeDee:segmentationMaskOpacity", "Segmentation Mask Opacity"),
          input: "number",
          min: 0,
          max: 1,
          step: 0.1,
          precision: 2,
          placeholder: String(0.5), // Default value
          value: config.segmentationMaskOpacity,
        },
      };

      entries.push({
        path: ["topics", imageTopic],
        node: {
          icon: "ImageProjection",
          fields,
          visible: config.visible ?? IMAGE_RENDERABLE_DEFAULT_SETTINGS.visible,
          order: imageTopic.toLocaleLowerCase(),
          handler,
        },
      });
    }
    return entries;
  }

  public override handleSettingsAction = (action: SettingsTreeAction): void => {
    const path = action.payload.path;
    if (action.action !== "update" || path.length !== 3) {
      return;
    }

    const imageTopic = path[1]!;
    const imageTopic = path[1]!;
    const property = path[2] as keyof LayerSettingsImage | undefined;

    const prevSettings = this.renderer.config.topics[imageTopic] as Partial<LayerSettingsImage> | undefined;
    const prevCameraInfoTopic = prevSettings?.cameraInfoTopic;
    const prevSegmentationMaskTopic = prevSettings?.segmentationMaskTopic;

    this.saveSetting(path, action.payload.value);

    const settings = this.renderer.config.topics[imageTopic] as Partial<LayerSettingsImage> | undefined;
    const cameraInfoTopic = settings?.cameraInfoTopic;
    const segmentationMaskTopic = settings?.segmentationMaskTopic;

    // Update cameraInfoTopic mapping
    if (cameraInfoTopic !== prevCameraInfoTopic) {
      if (prevCameraInfoTopic != undefined) {
        this.#cameraInfoToImageTopics.delete(prevCameraInfoTopic, imageTopic);
      }
      if (cameraInfoTopic != undefined) {
        this.#cameraInfoToImageTopics.set(cameraInfoTopic, imageTopic);
      }
    }

    // Update segmentationMaskTopic mapping
    if (segmentationMaskTopic !== prevSegmentationMaskTopic) {
      if (prevSegmentationMaskTopic != undefined) {
        this.#segmentationMaskToImageTopics.delete(prevSegmentationMaskTopic, imageTopic);
      }
      if (segmentationMaskTopic != undefined) {
        this.#segmentationMaskToImageTopics.set(segmentationMaskTopic, imageTopic);
      }
    }
    
    const renderable = this.renderables.get(imageTopic);
    if (renderable) {
      renderable.setSettings({ ...IMAGE_RENDERABLE_DEFAULT_SETTINGS, ...settings });
    }

    // If visibility, cameraInfoTopic or segmentationMaskTopic changed, we might need to update subscriptions
    if (property === "visible" || property === "cameraInfoTopic" || property === "segmentationMaskTopic") {
      this.renderer.updateSubscriptions();
    }
    
    // Apply camera info if it's set and renderable exists
    if (renderable && cameraInfoTopic) {

    // Look up the camera info for our image topic
    const cameraInfo = this.#cameraInfoByTopic.get(cameraInfoTopic);
    if (!cameraInfo) {
      this.renderer.settings.errors.addToTopic(
        imageTopic,
        NO_CAMERA_INFO_ERR,
        `No CameraInfo received on ${cameraInfoTopic}`,
      );
      return;
    }
    this.#recomputeCameraModel(renderable, cameraInfo);
    renderable.update();
  };

  #cameraInfoShouldSubscribe = (cameraInfoTopic: string): boolean => {
    // Iterate over each topic config and check if it has a cameraInfoTopic setting that matches
    // the cameraInfoTopic we might want to turn on. If it does and the topic is visible, return true.
    for (const topicConfig of Object.values(this.renderer.config.topics)) {
      const imageConfig = topicConfig as Partial<LayerSettingsImage>;
      if (imageConfig.cameraInfoTopic === cameraInfoTopic && imageConfig.visible === true) {
        return true;
      }
    }
    return false;
  };

  #imageShouldSubscribe = (topicToSubscribe: string): boolean => {
    // Iterate over each configured image layer. A topic should be subscribed to if:
    // 1. It's the main image topic for a visible layer.
    // 2. It's the segmentationMaskTopic for a visible layer.
    for (const [imageTopicName, config] of Object.entries(this.renderer.config.topics)) {
      // Filter out non-image topics from config (e.g. if other extensions store topic config)
      if (!this.renderables.has(imageTopicName) && !topicIsConvertibleToSchema(this.renderer.topicFromName(imageTopicName), this.supportedImageSchemas)) {
          // If we don't have a renderable for this topic yet, and it's not an image schema, skip.
          // This check helps ensure we only consider image layers.
          // A more robust way might be to check if `config` has image-specific settings.
          if(!((config as Partial<LayerSettingsImage>).cameraInfoTopic !== undefined || (config as Partial<LayerSettingsImage>).distance !== undefined)){
            continue;
          }
      }

      const imageConfig = config as Partial<LayerSettingsImage>;
      if (imageConfig.visible === true) {
        if (imageTopicName === topicToSubscribe) {
          return true; // Main image topic for a visible layer
        }
        if (imageConfig.segmentationMaskTopic === topicToSubscribe) {
          return true; // Segmentation mask topic for a visible layer
        }
      }
    }
    return false;
  };
  
  #handleAnyImageMessage = (
    messageEvent: PartialMessageEvent<AnyImage>,
    normalizedImage: AnyImage,
  ): void => {
    const topic = messageEvent.topic;

    // Check if this topic is a main image for a renderable
    const mainRenderable = this.renderables.get(topic);
    if (mainRenderable) {
      this.#handleMainImage(messageEvent, normalizedImage, mainRenderable);
    }

    // Check if this topic is a segmentation mask for any renderables
    const imageTopicsUsingThisMask = this.#segmentationMaskToImageTopics.get(topic);
    if (imageTopicsUsingThisMask) {
      for (const imageTopic of imageTopicsUsingThisMask) {
        const renderable = this.renderables.get(imageTopic);
        if (renderable && renderable.userData.settings.segmentationMaskTopic === topic) {
          renderable.setSegmentationMaskImage(normalizedImage);
        }
      }
    }
  };
  
  // Renamed from handleImage to #handleMainImage and takes renderable as argument
  #handleMainImage = (
    messageEvent: PartialMessageEvent<AnyImage>,
    image: AnyImage,
    renderable: ImageRenderable, // Pass the renderable directly
  ): void => {
    renderable.userData.receiveTime = toNanoSec(messageEvent.receiveTime);
    renderable.setImage(image, DEFAULT_BITMAP_WIDTH);

    const imageTopic = renderable.userData.topic; // Use renderable's topic
    const settings = renderable.userData.settings;

    // Auto-select settings.cameraInfoTopic if it's not already set
    if (settings.cameraInfoTopic == undefined) {
      const prefix = getTopicMatchPrefix(imageTopic);
      const newCameraInfoTopic =
        prefix != undefined
          ? filterMap(this.#cameraInfoTopics, (topicName) => // Use topicName from #cameraInfoTopics
              topicName.startsWith(prefix) ? topicName : undefined,
            ).sort()[0]
          : undefined;
      
      if (newCameraInfoTopic) {
        settings.cameraInfoTopic = newCameraInfoTopic;
        // Update config reflectively:
        this.renderer.updateConfig((draft) => {
          const draftSettings = draft.topics[imageTopic] as Partial<LayerSettingsImage> | undefined;
          if (draftSettings) {
            draftSettings.cameraInfoTopic = newCameraInfoTopic;
          }
        });
        // renderable.setSettings(settings) will be called by handleSettingsAction or after this block
        // No, we need to call it here or ensure the new settings object is used.
        renderable.setSettings({ ...settings }); // Pass a new object to trigger updates
        this.updateSettingsTree(); // Reflect change in UI
         if (!this.#cameraInfoToImageTopics.has(newCameraInfoTopic, imageTopic)) {
           this.#cameraInfoToImageTopics.set(newCameraInfoTopic, imageTopic);
         }
      } else {
        this.renderer.settings.errors.addToTopic(
          imageTopic,
          NO_CAMERA_INFO_ERR,
          `No CameraInfo topic found matching prefix of ${imageTopic}`,
        );
        // Don't return yet, image can still be shown without camera info (e.g. planar projection)
      }
    }

    // Apply camera info if available
    if (settings.cameraInfoTopic) {
      const cameraInfo = this.#cameraInfoByTopic.get(settings.cameraInfoTopic);
      if (!cameraInfo) {
        this.renderer.settings.errors.addToTopic(
          imageTopic,
          NO_CAMERA_INFO_ERR,
          `No CameraInfo received on ${settings.cameraInfoTopic}`,
        );
      } else {
        this.renderer.settings.errors.removeFromTopic(imageTopic, NO_CAMERA_INFO_ERR);
        this.#recomputeCameraModel(renderable, cameraInfo);
      }
    } else {
       // If no camera info topic is set, ensure we clear any old camera model / errors
       renderable.setCameraModel(undefined); // Clears camera model
       renderable.userData.cameraInfo = undefined;
       this.renderer.settings.errors.removeFromTopic(imageTopic, NO_CAMERA_INFO_ERR);
       this.renderer.settings.errors.removeFromTopic(imageTopic, CAMERA_MODEL);
    }
    renderable.update(); // Ensure renderable updates itself
  };

  #handleCameraInfo = (
    messageEvent: PartialMessageEvent<CameraInfo> & PartialMessageEvent<CameraCalibration>,
  ): void => {
    // Store the last camera info on each topic, when processing an image message we'll look up
    // the camera info by the info topic configured for the image
    const cameraInfo = normalizeCameraInfo(messageEvent.message);
    this.#cameraInfoByTopic.set(messageEvent.topic, cameraInfo);

    // Look up any image topics assigned to our camera info topic and determine if we need to update
    // those renderables since we now have a camera info whereas we may not have previously
    const imageTopics = this.#cameraInfoToImageTopics.get(messageEvent.topic) ?? [];
    for (const imageTopic of imageTopics) {
      const renderable = this.renderables.get(imageTopic);
      if (!renderable) {
        continue;
      }

      // If there's no camera info topic assigned then we don't need to do update this renderable
      const settings = renderable.userData.settings;
      if (!settings.cameraInfoTopic || settings.cameraInfoTopic !== messageEvent.topic) {
        continue;
      }
      this.renderer.settings.errors.removeFromTopic(imageTopic, NO_CAMERA_INFO_ERR);

      this.#recomputeCameraModel(renderable, cameraInfo);
      renderable.update();
    }
  };

  /**
   * Recompute a new camera model if the newCameraInfo differs from the current renderable info. If
   * the info is unchanged then the existing camera model is returned.
   *
   * If a camera model could not be created this returns undefined.
   *
   * This function will set a topic error on the image topic if the camera model creation fails.
   */
  #recomputeCameraModel(renderable: ImageRenderable, newCameraInfo: CameraInfo) {
    // If the camera info has not changed, we don't need to make a new model and can return the existing one
    const dataEqual = cameraInfosEqual(renderable.userData.cameraInfo, newCameraInfo);
    if (dataEqual && renderable.userData.cameraModel != undefined) {
      return;
    }

    const imageTopic = renderable.userData.topic;

    try {
      renderable.setCameraModel(new PinholeCameraModel(newCameraInfo));
      renderable.userData.cameraInfo = newCameraInfo;
      this.renderer.settings.errors.removeFromTopic(imageTopic, CAMERA_MODEL);
    } catch (errUnk) {
      const err = errUnk as Error;
      this.renderer.settings.errors.addToTopic(imageTopic, CAMERA_MODEL, err.message);
    }
  }

  // Get or create an image renderable for the imageTopic
  #getImageRenderable(
    imageTopic: string,
    receiveTime: bigint,
    image: AnyImage | undefined, // The first image message, undefined if called before any image is received
    frameId: string, // frameId from the first image message
  ): ImageRenderable {
    let renderable = this.renderables.get(imageTopic);
    if (renderable) {
      return renderable;
    }

    const userSettings = (this.renderer.config.topics[imageTopic] ?? {}) as Partial<LayerSettingsImage>;
    const initialSettings = { ...IMAGE_RENDERABLE_DEFAULT_SETTINGS, ...userSettings };
    
    const messageTime = image ? toNanoSec("header" in image ? image.header.stamp : image.timestamp) : 0n;

    renderable = this.initRenderable(imageTopic, {
      receiveTime,
      messageTime,
      firstMessageTime: messageTime,
      frameId: this.renderer.normalizeFrameId(frameId),
      pose: makePose(),
      settingsPath: ["topics", imageTopic],
      topic: imageTopic,
      settings: initialSettings, // Use combined settings
      cameraInfo: undefined,
      cameraModel: undefined,
      image, // Initial image
      texture: undefined,
      material: undefined, // Material will be initialized by ImageRenderable constructor
      geometry: undefined,
      mesh: undefined,
      // Explicitly initialize new fields from ImageUserData related to segmentation masks
      segmentationMaskImage: undefined,
      segmentationMaskIndexTexture: undefined,
      segmentationMaskColorMapTexture: undefined,
      maxMaskIndex: 0,
      classColorMap: new Map(),
    });

    this.add(renderable);
    this.renderables.set(imageTopic, renderable);

    // Populate bidirectional maps after renderable creation
    if (initialSettings.cameraInfoTopic) {
      this.#cameraInfoToImageTopics.set(initialSettings.cameraInfoTopic, imageTopic);
    }
    if (initialSettings.segmentationMaskTopic) {
      this.#segmentationMaskToImageTopics.set(initialSettings.segmentationMaskTopic, imageTopic);
    }

    return renderable;
  }

  // Override to create renderables for topics not yet in this.renderables,
  // if they appear in settings (e.g. added by user but no message received yet)
  public override syncSettings(): void {
    super.syncSettings(); // Handles removal of renderables for topics no longer in settings

    // Create renderables for new topics in settings
    for (const topicName of Object.keys(this.renderer.config.topics)) {
        if (!this.renderables.has(topicName) && topicIsConvertibleToSchema(this.renderer.topicFromName(topicName), this.supportedImageSchemas)) {
            // Create a placeholder renderable. frameId and image will be set on first message.
            // receiveTime and messageTime are set to 0n, will be updated on first message.
            const renderable = this.#getImageRenderable(topicName, 0n, undefined, "");
            // Apply settings again in case #getImageRenderable used defaults before full config was available
            const userSettings = (this.renderer.config.topics[topicName] ?? {}) as Partial<LayerSettingsImage>;
            renderable.setSettings({ ...IMAGE_RENDERABLE_DEFAULT_SETTINGS, ...userSettings});
        }
    }
  }

  protected initRenderable(topicName: string, userData: ImageUserData): ImageRenderable {
    return new ImageRenderable(topicName, this.renderer, userData);
  }
}
