// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import * as _ from "lodash-es";
import * as THREE from "three";
import { assert } from "ts-essentials";

import { PinholeCameraModel } from "@foxglove/den/image";
import { VideoPlayer } from "@foxglove/den/video";
import Logger from "@foxglove/log";
import { toNanoSec } from "@foxglove/rostime";
import { IRenderer } from "@foxglove/studio-base/panels/ThreeDeeRender/IRenderer";
import { BaseUserData, Renderable } from "@foxglove/studio-base/panels/ThreeDeeRender/Renderable";
import { stringToRgba } from "@foxglove/studio-base/panels/ThreeDeeRender/color";
import { WorkerImageDecoder } from "@foxglove/studio-base/panels/ThreeDeeRender/renderables/Images/WorkerImageDecoder";
import { projectPixel } from "@foxglove/studio-base/panels/ThreeDeeRender/renderables/projections";
import { RosValue } from "@foxglove/studio-base/players/types";

import { AnyImage, CompressedVideo } from "./ImageTypes";
import {
  decodeCompressedImageToBitmap,
  decodeCompressedVideoToBitmap,
  emptyVideoFrame,
  getVideoDecoderConfig,
} from "./decodeImage";
import { CameraInfo } from "../../ros";
import { DECODE_IMAGE_ERR_KEY, IMAGE_TOPIC_PATH } from "../ImageMode/constants";
import { ColorModeSettings } from "../colorMode";

const log = Logger.getLogger(__filename);
export interface ImageRenderableSettings extends Partial<ColorModeSettings> {
  visible: boolean;
  frameLocked?: boolean;
  cameraInfoTopic: string | undefined;
  distance: number;
  planarProjectionFactor: number;
  color: string;
  // Added for segmentation mask
  segmentationMaskTopic: string | undefined;
  segmentationMaskOpacity: number;
}

const DEFAULT_DISTANCE = 1;
const DEFAULT_PLANAR_PROJECTION_FACTOR = 0;
export const IMAGE_RENDERABLE_DEFAULT_SETTINGS: ImageRenderableSettings = {
  visible: false,
  frameLocked: true,
  cameraInfoTopic: undefined,
  distance: DEFAULT_DISTANCE,
  planarProjectionFactor: DEFAULT_PLANAR_PROJECTION_FACTOR,
  color: "#ffffff",
  // Added for segmentation mask
  segmentationMaskTopic: undefined,
  segmentationMaskOpacity: 0.5, // Default opacity
};

const IMAGE_FORMATS = new Set(["jpeg", "jpg", "png", "webp"]);
const VIDEO_FORMATS = new Set(["h264"]);

export type ImageUserData = BaseUserData & {
  topic: string;
  settings: ImageRenderableSettings;
  firstMessageTime: bigint | undefined;
  cameraInfo: CameraInfo | undefined;
  cameraModel: PinholeCameraModel | undefined;
  image: AnyImage | undefined;
  texture: THREE.Texture | undefined;
  material: THREE.ShaderMaterial | THREE.MeshBasicMaterial | undefined; // Can be ShaderMaterial for masks
  geometry: THREE.PlaneGeometry | undefined;
  mesh: THREE.Mesh | undefined;
  // Added for segmentation mask
  segmentationMaskImage: AnyImage | undefined; // The raw mask image
  segmentationMaskIndexTexture: THREE.DataTexture | undefined; // Texture from raw mask data (indices)
  segmentationMaskColorMapTexture: THREE.DataTexture | undefined; // Texture for color mapping
  maxMaskIndex: number; // Max index found in the current mask
  classColorMap: Map<number, THREE.Color>; // Stores colors for class indices
};

// Helper to create a 1x1 placeholder texture
function createPlaceholderTexture(color = new THREE.Color(0x000000), alpha = 0.0): THREE.DataTexture {
  const data = new Uint8Array([color.r * 255, color.g * 255, color.b * 255, alpha * 255]);
  const texture = new THREE.DataTexture(data, 1, 1, THREE.RGBAFormat, THREE.UnsignedByteType);
  texture.needsUpdate = true;
  return texture;
}

const defaultPlaceholderTexture = createPlaceholderTexture();
const fullyTransparentPlaceholderTexture = createPlaceholderTexture(new THREE.Color(0x000000), 0.0);


// Shaders for segmentation mask overlay
const vertexShader = `
  varying vec2 vUv;
  void main() {
    vUv = uv;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }
`;

const fragmentShader = `
  varying vec2 vUv;
  uniform sampler2D uMainTexture;
  uniform sampler2D uMaskIndexTexture;      // Texture with class indices (e.g., R channel)
  uniform sampler2D uMaskColorMapTexture; // 1D texture mapping index to color
  uniform float uMaskOpacity;
  uniform float uMaxMaskIndex;            // Max index value in uMaskIndexTexture, for normalization
  uniform bool  uHasMask;                 // Indicates if a valid mask is present
  uniform vec3 uTintColor;                // Tint color from original settings.color
  uniform float uTintAlpha;               // Alpha from original settings.color (used for main image opacity)


  void main() {
    vec4 mainColor = texture2D(uMainTexture, vUv);
    mainColor.rgb *= uTintColor; // Apply original tint
    mainColor.a *= uTintAlpha;   // Apply original alpha

    if (!uHasMask || uMaskOpacity == 0.0) {
      gl_FragColor = mainColor;
      return;
    }

    // Sample the mask index (assuming it's in the R channel of uMaskIndexTexture)
    // The mask index texture should be configured with THREE.NearestFilter
    float maskIndex = texture2D(uMaskIndexTexture, vUv).r * 255.0; // Assuming uMaskIndexTexture stores raw uint8 indices

    if (maskIndex > uMaxMaskIndex) { // If index is out of bounds for the colormap
        gl_FragColor = mainColor;
        return;
    }

    // Normalize the index to use as a UV coordinate for the color map texture
    // Add 0.5 to sample the center of the texel.
    float normalizedIndexU = (maskIndex + 0.5) / (uMaxMaskIndex + 1.0);
    vec2 colorMapUv = vec2(normalizedIndexU, 0.5);

    vec4 maskMappedColor = texture2D(uMaskColorMapTexture, colorMapUv);

    // Blend the main image color with the mask color
    // The mask color should be fully opaque for mixing, then apply overall opacity.
    gl_FragColor = mix(mainColor, vec4(maskMappedColor.rgb, mainColor.a), uMaskOpacity);
  }
`;

export class ImageRenderable extends Renderable<ImageUserData> {
  // A lazily instantiated player for compressed video
  public videoPlayer: VideoPlayer | undefined;

  // Make sure that everything is build the first time we render
  // set when camera info or image changes
  #geometryNeedsUpdate = true;
  // set when geometry or material reference changes
  #meshNeedsUpdate = true;
  // set when image changes
  #textureNeedsUpdate = true;
  // set when material or texture changes
  #materialNeedsUpdate = true;

  #renderBehindScene: boolean = false;

  #isUpdating = false;

  #decodedImage?: ImageBitmap | ImageData;
  protected decoder?: WorkerImageDecoder;
  #receivedImageSequenceNumber = 0;
  #displayedImageSequenceNumber = 0;
  #showingErrorImage = false;

  #disposed = false;

  public constructor(topicName: string, renderer: IRenderer, userData: ImageUserData) {
    super(topicName, renderer, {
      ...userData,
      // Initialize new fields for segmentation mask
      segmentationMaskTopic: userData.settings.segmentationMaskTopic,
      segmentationMaskOpacity: userData.settings.segmentationMaskOpacity,
      segmentationMaskImage: undefined,
      segmentationMaskIndexTexture: undefined,
      segmentationMaskColorMapTexture: undefined,
      maxMaskIndex: 0,
      classColorMap: new Map<number, THREE.Color>(),
    });

    // Ensure material is initialized (it will be MeshBasicMaterial by default)
    this.#initMaterial();
  }

  protected isDisposed(): boolean {
    return this.#disposed;
  }

  public getDecodedImage(): ImageBitmap | ImageData | undefined {
    return this.#decodedImage;
  }

  public override dispose(): void {
    this.#disposed = true;
    this.userData.texture?.dispose();
    if (this.userData.material) {
      if (this.userData.material instanceof THREE.ShaderMaterial) {
        // Dispose uniforms if they are textures
        Object.values(this.userData.material.uniforms).forEach((uniform) => {
          if (uniform.value instanceof THREE.Texture) {
            uniform.value.dispose();
          }
        });
      }
      this.userData.material.dispose();
    }
    this.userData.geometry?.dispose();
    // Dispose segmentation mask textures
    this.userData.segmentationMaskIndexTexture?.dispose();
    this.userData.segmentationMaskColorMapTexture?.dispose();
    this.decoder?.terminate();
    super.dispose();
  }

  public updateHeaderInfo(): void {
    assert(this.userData.image, "updateHeaderInfo called without image");

    // If there is camera info, the frameId comes from the camera info since the user may have
    // selected camera info with a different frame than our image frame.
    //
    // If there is no camera info, we fall back to the image's frame
    const image = this.userData.image;
    const rawFrameId =
      this.userData.cameraInfo?.header.frame_id ??
      ("header" in image ? image.header.frame_id : image.frame_id);
    this.userData.frameId =
      typeof rawFrameId === "string" ? this.renderer.normalizeFrameId(rawFrameId) : rawFrameId;
    this.userData.messageTime = toNanoSec("header" in image ? image.header.stamp : image.timestamp);
  }

  public override details(): Record<string, RosValue> {
    return { image: this.userData.image, camera_info: this.userData.cameraInfo };
  }

  public setRenderBehindScene(): void {
    this.#renderBehindScene = true;
    this.#materialNeedsUpdate = true;
    this.#meshNeedsUpdate = true;
  }

  // Renderable should only need to care about the model
  public setCameraModel(cameraModel: PinholeCameraModel): void {
    this.#geometryNeedsUpdate ||= this.userData.cameraModel !== cameraModel;
    this.userData.cameraModel = cameraModel;
  }

  public setSettings(newSettings: ImageRenderableSettings): void {
    const prevSettings = this.userData.settings;
    if (prevSettings.cameraInfoTopic !== newSettings.cameraInfoTopic) {
      // clear mesh since it is no longer showing userData accurately
      if (this.userData.mesh != undefined) {
        this.remove(this.userData.mesh);
      }
      this.userData.mesh = undefined;
      this.#geometryNeedsUpdate = true;
    }
    if (
      prevSettings.distance !== newSettings.distance ||
      newSettings.planarProjectionFactor !== prevSettings.planarProjectionFactor
    ) {
      this.#geometryNeedsUpdate = true;
    }

    if (newSettings.color !== prevSettings.color) {
      this.#materialNeedsUpdate = true;
    }

    // Handle segmentation mask settings changes
    if (
      newSettings.segmentationMaskTopic !== prevSettings.segmentationMaskTopic ||
      newSettings.segmentationMaskOpacity !== prevSettings.segmentationMaskOpacity
    ) {
      // If the topic changes, we might need to clear the old mask image/texture
      if (newSettings.segmentationMaskTopic !== prevSettings.segmentationMaskTopic) {
        this.userData.segmentationMaskImage = undefined;
        this.userData.segmentationMaskIndexTexture?.dispose();
        this.userData.segmentationMaskIndexTexture = undefined;
        this.userData.segmentationMaskColorMapTexture?.dispose();
        this.userData.segmentationMaskColorMapTexture = undefined;
        this.userData.maxMaskIndex = 0;
        this.userData.classColorMap.clear();
        // Future: Trigger re-subscription if necessary (likely handled by Images.ts)
      }
      // We need to update the material or shader if opacity changes, mark for update.
      this.#materialNeedsUpdate = true;
    }

    if (
      prevSettings.colorMode !== newSettings.colorMode ||
      prevSettings.flatColor !== newSettings.flatColor ||
      !_.isEqual(prevSettings.gradient, newSettings.gradient) ||
      prevSettings.colorMap !== newSettings.colorMap ||
      prevSettings.minValue !== newSettings.minValue ||
      prevSettings.maxValue !== newSettings.maxValue
    ) {
      this.userData.settings = newSettings;
      // Decode the current image again, which takes into account the new options
      const image = this.userData.image;
      if (image) {
        this.setImage(image);
      }
      return;
    }

    this.userData.settings = newSettings;
    // Update userData directly with new segmentation settings
    this.userData.segmentationMaskTopic = newSettings.segmentationMaskTopic;
    this.userData.segmentationMaskOpacity = newSettings.segmentationMaskOpacity;
  }

  public setImage(image: AnyImage, resizeWidth?: number, onDecoded?: () => void): void {
    this.userData.image = image;

    const seq = ++this.#receivedImageSequenceNumber;
    const decodePromise = this.decodeImage(image, resizeWidth);

    decodePromise
      .then((result) => {
        if (this.isDisposed()) {
          return;
        }
        // prevent displaying an image older than the one currently displayed
        if (this.#displayedImageSequenceNumber > seq) {
          return;
        }
        this.#displayedImageSequenceNumber = seq;
        this.#decodedImage = result;
        this.#textureNeedsUpdate = true;
        this.update();
        this.#showingErrorImage = false;

        onDecoded?.();
        this.removeError(DECODE_IMAGE_ERR_KEY);
        this.renderer.queueAnimationFrame();
      })
      .catch((err) => {
        log.error(err);
        if (this.isDisposed()) {
          return;
        }
        // avoid needing to recreate error image if it already shown
        if (!this.#showingErrorImage) {
          void this.#setErrorImage(seq, onDecoded);
        }
        this.addError(DECODE_IMAGE_ERR_KEY, `Error decoding image: ${err.message}`);
      });
  }

  async #setErrorImage(seq: number, onDecoded?: () => void): Promise<void> {
    const errorBitmap = await getErrorImage(64, 64);
    if (this.isDisposed()) {
      return;
    }
    if (this.#displayedImageSequenceNumber > seq) {
      return;
    }
    this.#decodedImage = errorBitmap;
    this.#textureNeedsUpdate = true;
    this.update();
    this.#showingErrorImage = true;
    // call ondecoded to display the error image when calibration is None
    onDecoded?.();
    this.renderer.queueAnimationFrame();
  }

  protected async decodeImage(
    image: AnyImage,
    resizeWidth?: number,
  ): Promise<ImageBitmap | ImageData> {
    if ("format" in image) {
      if (VIDEO_FORMATS.has(image.format)) {
        const frameMsg = image as CompressedVideo;

        if (frameMsg.data.byteLength === 0) {
          const error = "Empty video frame";
          log.error(error);
          // show last frame instead of error image if available
          if (this.videoPlayer?.lastImageBitmap) {
            return this.videoPlayer.lastImageBitmap;
          }
          // show black image instead of error image
          return await emptyVideoFrame(this.videoPlayer, resizeWidth);
          // Raise error so the caller can catch it and display an error image
          throw new Error(error);
        }

        if (!this.videoPlayer) {
          this.videoPlayer = new VideoPlayer();
          this.videoPlayer.on("error", (err) => {
            log.error(err);
            this.addError(DECODE_IMAGE_ERR_KEY, `Error decoding video: ${err.message}`);
          });
          this.videoPlayer.on("warn", (msg) => {
            log.warn(msg);
          });
        }
        const videoPlayer = this.videoPlayer;

        // Initialize the video player if needed
        if (!videoPlayer.isInitialized()) {
          const decoderConfig = getVideoDecoderConfig(frameMsg);
          if (decoderConfig) {
            await videoPlayer.init(decoderConfig);
          } else {
            // Raise error so the caller can catch it
            throw new Error("Waiting for keyframe");
            return await emptyVideoFrame(this.videoPlayer, resizeWidth);
          }
        }

        assert(this.userData.firstMessageTime != undefined, "firstMessageTime must be set");

        return await decodeCompressedVideoToBitmap(
          frameMsg,
          videoPlayer,
          this.userData.firstMessageTime,
          resizeWidth,
        );
      } else if (IMAGE_FORMATS.has(image.format)) {
        return await decodeCompressedImageToBitmap(image, resizeWidth);
      } else {
        // Raise error so the caller can catch it
        throw new Error(`Unsupported format: "${image.format}"`);
      }
    }
    return await (this.decoder ??= new WorkerImageDecoder()).decode(image, this.userData.settings);
  }

  public update(): void {
    if (this.#isUpdating) {
      return;
    }
    this.#isUpdating = true;

    if (this.#textureNeedsUpdate && this.#decodedImage) {
      this.#updateTexture();
      this.#textureNeedsUpdate = false;
    }

    if (this.userData.image) {
      this.updateHeaderInfo();
    }

    if (this.#geometryNeedsUpdate && this.userData.cameraModel) {
      this.#rebuildGeometry();
      this.#geometryNeedsUpdate = false;
    }

    if (this.#materialNeedsUpdate) {
      this.#updateMaterial();
      this.#materialNeedsUpdate = false;
    }

    if (
      this.#meshNeedsUpdate &&
      this.userData.texture &&
      this.userData.geometry &&
      this.userData.material
    ) {
      this.#updateMesh();
      this.#meshNeedsUpdate = false;
    }
    this.#isUpdating = false;
  }

  #rebuildGeometry() {
    assert(this.userData.cameraModel, "Camera model must be set before geometry can be updated");
    // Dispose of the current geometry if the settings have changed
    this.userData.geometry?.dispose();
    this.userData.geometry = undefined;
    const geometry = createGeometry(this.userData.cameraModel, this.userData.settings);
    this.userData.geometry = geometry;
    this.#meshNeedsUpdate = true;
  }

  #updateTexture(): void {
    assert(
      this.#decodedImage,
      "Decoded image must be set before texture can be updated or created",
    );
    const decodedImage = this.#decodedImage;
    // Create or update the bitmap texture
    if (decodedImage instanceof ImageBitmap) {
      const canvasTexture = this.userData.texture;
      if (
        canvasTexture == undefined ||
        // instanceof check allows us to switch from a raw image (DataTexture) to a compressed image (CanvasTexture)
        !(canvasTexture instanceof THREE.CanvasTexture) ||
        !bitmapDimensionsEqual(decodedImage, canvasTexture.image as ImageBitmap | undefined)
      ) {
        if (canvasTexture?.image instanceof ImageBitmap) {
          // don't close the image if it is the error image
          canvasTexture.image.close();
        }
        canvasTexture?.dispose();
        this.userData.texture = createCanvasTexture(decodedImage);
      } else {
        canvasTexture.image = decodedImage;
        canvasTexture.needsUpdate = true;
      }
    } else {
      let dataTexture = this.userData.texture;
      if (
        dataTexture == undefined ||
        // instanceof check allows us to switch from a compressed image (CanvasTexture) to a raw image (DataTexture)
        !(dataTexture instanceof THREE.DataTexture) ||
        dataTexture.image.width !== decodedImage.width ||
        dataTexture.image.height !== decodedImage.height
      ) {
        dataTexture?.dispose();
        dataTexture = createDataTexture(decodedImage);
        this.userData.texture = dataTexture;
      } else {
        dataTexture.image = decodedImage;
        dataTexture.needsUpdate = true;
      }
    }
    this.#materialNeedsUpdate = true;
  }

  #updateMaterial(): void {
    const useShaderMaterial =
      this.userData.segmentationMaskImage && this.userData.settings.segmentationMaskOpacity > 0;

    if (useShaderMaterial) {
      if (!(this.userData.material instanceof THREE.ShaderMaterial)) {
        this.userData.material?.dispose(); // Dispose old MeshBasicMaterial
        this.#initShaderMaterial();
        this.#meshNeedsUpdate = true; // Mesh needs to be updated with new material instance
      }

      const material = this.userData.material as THREE.ShaderMaterial;
      material.uniforms.uMainTexture.value = this.userData.texture ?? defaultPlaceholderTexture;
      material.uniforms.uMaskIndexTexture.value = this.userData.segmentationMaskIndexTexture ?? fullyTransparentPlaceholderTexture;
      material.uniforms.uMaskColorMapTexture.value = this.userData.segmentationMaskColorMapTexture ?? fullyTransparentPlaceholderTexture;
      material.uniforms.uMaskOpacity.value = this.userData.settings.segmentationMaskOpacity;
      material.uniforms.uMaxMaskIndex.value = this.userData.maxMaskIndex;
      material.uniforms.uHasMask.value = this.userData.segmentationMaskIndexTexture != undefined && this.userData.segmentationMaskColorMapTexture != undefined;

      stringToRgba(tempColor, this.userData.settings.color);
      material.uniforms.uTintColor.value.setRGB(tempColor.r, tempColor.g, tempColor.b);
      material.uniforms.uTintAlpha.value = tempColor.a;
      
      const transparent = tempColor.a < 1 || this.userData.settings.segmentationMaskOpacity < 1;
      material.transparent = transparent;
      material.depthWrite = !transparent;


    } else {
      if (this.userData.material instanceof THREE.ShaderMaterial) {
        // Dispose ShaderMaterial and its textures if switching back to MeshBasicMaterial
        Object.values(this.userData.material.uniforms).forEach((uniform) => {
          if (uniform.value instanceof THREE.Texture && uniform.value !== defaultPlaceholderTexture && uniform.value !== fullyTransparentPlaceholderTexture) {
            // Do not dispose shared placeholder textures
          }
        });
        this.userData.material.dispose();
        this.userData.material = undefined; // Force re-initialization
      }
      if (!this.userData.material) {
        this.#initBasicMaterial();
        this.#meshNeedsUpdate = true; // Mesh needs to be updated with new material instance
      }
      
      const material = this.userData.material as THREE.MeshBasicMaterial;
      material.map = this.userData.texture ?? null;

      stringToRgba(tempColor, this.userData.settings.color);
      const transparent = tempColor.a < 1;
      material.color.setRGB(tempColor.r, tempColor.g, tempColor.b);
      material.opacity = tempColor.a;
      material.transparent = transparent;
      material.depthWrite = !transparent;
    }
    
    const currentMaterial = this.userData.material!;
    if (this.#renderBehindScene) {
      currentMaterial.depthWrite = false;
      currentMaterial.depthTest = false;
    } else {
      currentMaterial.depthTest = true;
    }
    currentMaterial.needsUpdate = true;
  }

  #initBasicMaterial(): void {
    stringToRgba(tempColor, this.userData.settings.color);
    const transparent = tempColor.a < 1;
    const color = new THREE.Color(tempColor.r, tempColor.g, tempColor.b);
    this.userData.material = new THREE.MeshBasicMaterial({
      name: `${this.userData.topic}:BasicMaterial`,
      color,
      side: THREE.DoubleSide,
      opacity: tempColor.a,
      transparent,
      depthWrite: !transparent,
      map: this.userData.texture ?? null,
    });
  }

  #initShaderMaterial(): void {
    stringToRgba(tempColor, this.userData.settings.color);
    const transparent = tempColor.a < 1 || this.userData.settings.segmentationMaskOpacity < 1;

    this.userData.material = new THREE.ShaderMaterial({
      name: `${this.userData.topic}:ShaderMaterial`,
      vertexShader,
      fragmentShader,
      uniforms: {
        uMainTexture: { value: this.userData.texture ?? defaultPlaceholderTexture },
        uMaskIndexTexture: { value: this.userData.segmentationMaskIndexTexture ?? fullyTransparentPlaceholderTexture },
        uMaskColorMapTexture: { value: this.userData.segmentationMaskColorMapTexture ?? fullyTransparentPlaceholderTexture },
        uMaskOpacity: { value: this.userData.settings.segmentationMaskOpacity },
        uMaxMaskIndex: { value: this.userData.maxMaskIndex },
        uHasMask: { value: false }, // Will be updated in #updateMaterial
        uTintColor: { value: new THREE.Color(tempColor.r, tempColor.g, tempColor.b) },
        uTintAlpha: { value: tempColor.a },
      },
      side: THREE.DoubleSide,
      transparent,
      depthWrite: !transparent,
    });
  }
  
  #initMaterial(): void {
    // Default to basic material, #updateMaterial will switch if needed
    this.#initBasicMaterial();
  }


  #updateMesh(): void {
    assert(this.userData.geometry, "Geometry must be set before mesh can be updated or created");
    assert(this.userData.material, "Material must be set before mesh can be updated or created");
    if (!this.userData.mesh) {
      this.userData.mesh = new THREE.Mesh(this.userData.geometry, this.userData.material);
      this.add(this.userData.mesh);
    } else {
      this.userData.mesh.geometry = this.userData.geometry;
      this.userData.mesh.material = this.userData.material;
    }

    if (!this.#renderBehindScene) {
      this.userData.mesh.renderOrder = 0;
      return;
    }

    this.userData.mesh.renderOrder = -1 * Number.MAX_SAFE_INTEGER;
  }

  protected addError(key: string, message: string): void {
    if (this.isDisposed()) {
      return;
    }
    // must account for if the renderable is part of `ImageMode` or `Images` scene extension
    this.renderer.settings.errors.add(IMAGE_TOPIC_PATH, key, message);
    this.renderer.settings.errors.addToTopic(this.userData.topic, key, message);
  }

  protected removeError(key: string): void {
    this.renderer.settings.errors.remove(IMAGE_TOPIC_PATH, key);
    this.renderer.settings.errors.removeFromTopic(this.userData.topic, key);
  }

  public setSegmentationMaskImage(image: AnyImage): void {
    this.userData.segmentationMaskImage = image;

    if (!("encoding" in image && (image.encoding === "mono8" || image.encoding === "8UC1"))) {
      log.error(`Unsupported segmentation mask encoding: ${"encoding" in image ? image.encoding : "unknown"}. Only mono8/8UC1 is currently supported.`);
      this.userData.segmentationMaskIndexTexture?.dispose();
      this.userData.segmentationMaskIndexTexture = undefined;
      this.userData.segmentationMaskColorMapTexture?.dispose();
      this.userData.segmentationMaskColorMapTexture = undefined;
      this.userData.maxMaskIndex = 0;
      this.#materialNeedsUpdate = true;
      this.update();
      this.renderer.queueAnimationFrame();
      return;
    }

    const maskData = image.data; // Assuming Uint8Array for mono8
    const width = image.width;
    const height = image.height;

    // Create/Update segmentationMaskIndexTexture (raw mask indices)
    if (this.userData.segmentationMaskIndexTexture) {
      if (this.userData.segmentationMaskIndexTexture.image.width !== width || this.userData.segmentationMaskIndexTexture.image.height !== height) {
        this.userData.segmentationMaskIndexTexture.dispose();
        this.userData.segmentationMaskIndexTexture = undefined;
      }
    }
    if (!this.userData.segmentationMaskIndexTexture) {
      this.userData.segmentationMaskIndexTexture = new THREE.DataTexture(
        maskData, width, height, THREE.RedFormat, THREE.UnsignedByteType
      );
      this.userData.segmentationMaskIndexTexture.minFilter = THREE.NearestFilter;
      this.userData.segmentationMaskIndexTexture.magFilter = THREE.NearestFilter;
      this.userData.segmentationMaskIndexTexture.needsUpdate = true;
    } else {
      this.userData.segmentationMaskIndexTexture.image.data = maskData;
      this.userData.segmentationMaskIndexTexture.needsUpdate = true;
    }
    
    // Find unique class indices and max index
    let currentMaxIndex = 0;
    const uniqueIndices = new Set<number>();
    for (let i = 0; i < maskData.length; i++) {
      const index = maskData[i]!;
      uniqueIndices.add(index);
      if (index > currentMaxIndex) {
        currentMaxIndex = index;
      }
    }
    this.userData.maxMaskIndex = currentMaxIndex;

    // Generate colors for new indices
    uniqueIndices.forEach(index => {
      if (!this.userData.classColorMap.has(index)) {
        this.userData.classColorMap.set(index, new THREE.Color().setHSL(Math.random(), 0.7, 0.5));
      }
    });
    
    // Create/Update segmentationMaskColorMapTexture
    const colorMapWidth = this.userData.maxMaskIndex + 1;
    const colorMapData = new Uint8Array(colorMapWidth * 3); // RGB
    for (let i = 0; i <= this.userData.maxMaskIndex; i++) {
      const color = this.userData.classColorMap.get(i) ?? new THREE.Color(0x000000); // Default to black if not found
      colorMapData[i * 3 + 0] = color.r * 255;
      colorMapData[i * 3 + 1] = color.g * 255;
      colorMapData[i * 3 + 2] = color.b * 255;
    }

    if (this.userData.segmentationMaskColorMapTexture) {
       // If width changed, dispose and recreate
      if (this.userData.segmentationMaskColorMapTexture.image.width !== colorMapWidth) {
        this.userData.segmentationMaskColorMapTexture.dispose();
        this.userData.segmentationMaskColorMapTexture = undefined;
      }
    }

    if (!this.userData.segmentationMaskColorMapTexture) {
      this.userData.segmentationMaskColorMapTexture = new THREE.DataTexture(
        colorMapData, colorMapWidth, 1, THREE.RGBFormat, THREE.UnsignedByteType
      );
      this.userData.segmentationMaskColorMapTexture.minFilter = THREE.NearestFilter;
      this.userData.segmentationMaskColorMapTexture.magFilter = THREE.NearestFilter;
    } else {
      this.userData.segmentationMaskColorMapTexture.image.data = colorMapData;
      // Important: Update width/height if they changed, though height is fixed at 1
      this.userData.segmentationMaskColorMapTexture.image.width = colorMapWidth;
    }
    this.userData.segmentationMaskColorMapTexture.needsUpdate = true;

    this.#materialNeedsUpdate = true;
    this.update();
    this.renderer.queueAnimationFrame();
  }
}

let tempColor = { r: 0, g: 0, b: 0, a: 0 };

function createCanvasTexture(bitmap: ImageBitmap): THREE.CanvasTexture {
  const texture = new THREE.CanvasTexture(
    bitmap,
    THREE.UVMapping,
    THREE.ClampToEdgeWrapping,
    THREE.ClampToEdgeWrapping,
    THREE.NearestFilter,
    THREE.LinearFilter,
    THREE.RGBAFormat,
    THREE.UnsignedByteType,
  );
  texture.generateMipmaps = false;
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function createDataTexture(imageData: ImageData): THREE.DataTexture {
  const dataTexture = new THREE.DataTexture(
    imageData.data,
    imageData.width,
    imageData.height,
    THREE.RGBAFormat,
    THREE.UnsignedByteType,
    THREE.UVMapping,
    THREE.ClampToEdgeWrapping,
    THREE.ClampToEdgeWrapping,
    THREE.NearestFilter,
    THREE.LinearFilter,
    1,
    THREE.SRGBColorSpace,
  );
  dataTexture.needsUpdate = true; // ensure initial image data is displayed
  return dataTexture;
}

function createGeometry(
  cameraModel: PinholeCameraModel,
  settings: ImageRenderableSettings,
): THREE.PlaneGeometry {
  const WIDTH_SEGMENTS = 10;
  const HEIGHT_SEGMENTS = 10;

  const width = cameraModel.width;
  const height = cameraModel.height;
  const geometry = new THREE.PlaneGeometry(1, 1, WIDTH_SEGMENTS, HEIGHT_SEGMENTS);

  const gridX1 = WIDTH_SEGMENTS + 1;
  const gridY1 = HEIGHT_SEGMENTS + 1;
  const size = gridX1 * gridY1;

  const segmentWidth = width / WIDTH_SEGMENTS;
  const segmentHeight = height / HEIGHT_SEGMENTS;

  // Use a slight offset to avoid z-fighting with the CameraInfo wireframe
  const EPS = 1e-3;

  // Rebuild the position buffer for the plane by iterating through the grid and
  // projecting each pixel space x/y coordinate into a 3D ray and casting out by
  // the user-configured distance setting. UV coordinates are rebuilt so the
  // image is not vertically flipped
  const pixel = { x: 0, y: 0 };
  const p = { x: 0, y: 0, z: 0 };
  const vertices = new Float32Array(size * 3);
  const uvs = new Float32Array(size * 2);
  for (let iy = 0; iy < gridY1; iy++) {
    for (let ix = 0; ix < gridX1; ix++) {
      const vOffset = (iy * gridX1 + ix) * 3;
      const uvOffset = (iy * gridX1 + ix) * 2;

      pixel.x = ix * segmentWidth;
      pixel.y = iy * segmentHeight;
      projectPixel(p, pixel, cameraModel, settings);
      vertices[vOffset + 0] = p.x;
      vertices[vOffset + 1] = p.y;
      vertices[vOffset + 2] = p.z - EPS;

      uvs[uvOffset + 0] = ix / WIDTH_SEGMENTS;
      uvs[uvOffset + 1] = iy / HEIGHT_SEGMENTS;
    }
  }
  geometry.setAttribute("position", new THREE.BufferAttribute(vertices, 3));
  geometry.setAttribute("uv", new THREE.BufferAttribute(uvs, 2));
  geometry.attributes.position!.needsUpdate = true;
  geometry.attributes.uv!.needsUpdate = true;

  return geometry;
}

const bitmapDimensionsEqual = (a?: ImageBitmap, b?: ImageBitmap) =>
  a?.width === b?.width && a?.height === b?.height;

async function getErrorImage(width: number, height: number): Promise<ImageBitmap> {
  const canvas = document.createElement("canvas");
  const ctx = canvas.getContext("2d");
  if (!ctx) {
    throw Error("Could not instantiate 2D canvas context");
  }

  canvas.width = width;
  canvas.height = height;

  // Draw outline
  ctx.strokeStyle = "red";
  ctx.lineWidth = 2;
  ctx.strokeRect(0, 0, width, height);

  // Draw X
  ctx.strokeStyle = "red";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.lineTo(width, height);
  ctx.moveTo(width, 0);
  ctx.lineTo(0, height);
  ctx.stroke();

  // Get the updated image data
  const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
  const bitmap = await createImageBitmap(imageData, { resizeWidth: width });

  return bitmap;
}
