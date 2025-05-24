import { Meta, StoryObj } from "@storybook/react";
import React from "react";

import { RawImage } from "@foxglove/schemas";
import { MessageEvent, Time } from "@foxglove/studio";
import { Topic } from "@foxglove/studio-base/players/types";
import PanelSetup, { Fixture } from "@foxglove/studio-base/stories/PanelSetup";

import { ImagePanel } from "../../index";
import { LayerSettingsImage } from "../../renderables/Images";
import { RendererConfig } from "../../IRenderer";
import { DEFAULT_CAMERA_STATE, rad2deg } from "../../camera";

export default {
  title: "panels/ThreeDeeRender/Image Segmentation Overlay",
  component: ImagePanel,
  parameters: { colorScheme: "light" },
} as Meta;

const MAIN_IMAGE_TOPIC = "/test/main_image";
const MASK_TOPIC_1 = "/test/mask_1";
const MASK_TOPIC_2 = "/test/mask_2";
const MASK_TOPIC_NONEXISTENT = "/test/mask_nonexistent";

const DEFAULT_TIME: Time = { sec: 10, nsec: 0 };
const FRAME_ID = "test_frame";

// --- Mock Image Data ---
const IMAGE_WIDTH = 20;
const IMAGE_HEIGHT = 20;

// Main Image: 4 quadrants (R, G, B, W)
const mainImage: RawImage = (() => {
  const data = new Uint8Array(IMAGE_WIDTH * IMAGE_HEIGHT * 3);
  for (let y = 0; y < IMAGE_HEIGHT; y++) {
    for (let x = 0; x < IMAGE_WIDTH; x++) {
      const i = (y * IMAGE_WIDTH + x) * 3;
      if (y < IMAGE_HEIGHT / 2) {
        if (x < IMAGE_WIDTH / 2) { // Top-left: Red
          data[i] = 255; data[i + 1] = 0; data[i + 2] = 0;
        } else { // Top-right: Green
          data[i] = 0; data[i + 1] = 255; data[i + 2] = 0;
        }
      } else {
        if (x < IMAGE_WIDTH / 2) { // Bottom-left: Blue
          data[i] = 0; data[i + 1] = 0; data[i + 2] = 255;
        } else { // Bottom-right: White
          data[i] = 255; data[i + 1] = 255; data[i + 2] = 255;
        }
      }
    }
  }
  return {
    timestamp: DEFAULT_TIME,
    frame_id: FRAME_ID,
    width: IMAGE_WIDTH,
    height: IMAGE_HEIGHT,
    encoding: "rgb8",
    step: IMAGE_WIDTH * 3,
    data,
  };
})();

// Mask 1: Left half class 0, right half class 1
const segmentationMask1: RawImage = (() => {
  const data = new Uint8Array(IMAGE_WIDTH * IMAGE_HEIGHT);
  for (let y = 0; y < IMAGE_HEIGHT; y++) {
    for (let x = 0; x < IMAGE_WIDTH; x++) {
      data[y * IMAGE_WIDTH + x] = x < IMAGE_WIDTH / 2 ? 0 : 1;
    }
  }
  return {
    timestamp: DEFAULT_TIME,
    frame_id: FRAME_ID,
    width: IMAGE_WIDTH,
    height: IMAGE_HEIGHT,
    encoding: "mono8",
    step: IMAGE_WIDTH,
    data,
  };
})();

// Mask 2: Top third class 0, middle third class 1, bottom third class 2
const segmentationMask2: RawImage = (() => {
  const data = new Uint8Array(IMAGE_WIDTH * IMAGE_HEIGHT);
  for (let y = 0; y < IMAGE_HEIGHT; y++) {
    for (let x = 0; x < IMAGE_WIDTH; x++) {
      if (y < IMAGE_HEIGHT / 3) {
        data[y * IMAGE_WIDTH + x] = 0;
      } else if (y < (IMAGE_HEIGHT * 2) / 3) {
        data[y * IMAGE_WIDTH + x] = 1;
      } else {
        data[y * IMAGE_WIDTH + x] = 2;
      }
    }
  }
  return {
    timestamp: DEFAULT_TIME,
    frame_id: FRAME_ID,
    width: IMAGE_WIDTH,
    height: IMAGE_HEIGHT,
    encoding: "mono8",
    step: IMAGE_WIDTH,
    data,
  };
})();

// --- Message Event Creation ---
function createImageMessageEvent(
  topic: string,
  image: RawImage,
  timestamp: Time = DEFAULT_TIME,
): MessageEvent<RawImage> {
  return {
    topic,
    receiveTime: timestamp,
    message: image,
    schemaName: "foxglove.RawImage",
    sizeInBytes: image.data.length,
  };
}

const mainImageMessage = createImageMessageEvent(MAIN_IMAGE_TOPIC, mainImage);
const mask1Message = createImageMessageEvent(MASK_TOPIC_1, segmentationMask1);
const mask2Message = createImageMessageEvent(MASK_TOPIC_2, segmentationMask2);

// --- Story Component & Template ---
type ImageSegmentationOverlayStoryArgs = {
  segmentationMaskTopic?: string;
  segmentationMaskOpacity?: number;
  mainImageVisible?: boolean;
};

const ImageSegmentationOverlayStoryComponent: React.FC<ImageSegmentationOverlayStoryArgs> = ({
  segmentationMaskTopic,
  segmentationMaskOpacity,
  mainImageVisible = true,
}) => {
  const topics: Topic[] = [
    { name: MAIN_IMAGE_TOPIC, schemaName: "foxglove.RawImage" },
    { name: MASK_TOPIC_1, schemaName: "foxglove.RawImage" },
    { name: MASK_TOPIC_2, schemaName: "foxglove.RawImage" },
  ];

  const fixture: Fixture = {
    topics,
    frame: {
      [MAIN_IMAGE_TOPIC]: [mainImageMessage],
      [MASK_TOPIC_1]: [mask1Message],
      [MASK_TOPIC_2]: [mask2Message],
    },
    capabilities: [],
    activeData: { currentTime: DEFAULT_TIME },
  };

  const imageLayerSettings: Partial<LayerSettingsImage> = {
    visible: mainImageVisible,
  };
  if (segmentationMaskTopic !== undefined) {
    imageLayerSettings.segmentationMaskTopic = segmentationMaskTopic;
  }
  if (segmentationMaskOpacity !== undefined) {
    imageLayerSettings.segmentationMaskOpacity = segmentationMaskOpacity;
  }

  const config: DeepPartial<RendererConfig> = {
    followTf: FRAME_ID, // Follow the frame of the images
    scene: { enableStats: false },
    imageMode: {
      imageTopic: MAIN_IMAGE_TOPIC,
    },
    topics: {
      [MAIN_IMAGE_TOPIC]: imageLayerSettings,
    },
    cameraState: { // Simple camera state
      ...DEFAULT_CAMERA_STATE,
      distance: 25, // Zoom out a bit to see the 20x20 image clearly
      phi: rad2deg(Math.PI / 2), // Look straight down
      targetOffset: [0,0,0],
      thetaOffset: 0,
      fovy: 45,
    },
  };

  return (
    <PanelSetup fixture={fixture}>
      <ImagePanel overrideConfig={config as RendererConfig} />
    </PanelSetup>
  );
};

// --- Stories ---
export const BasicOverlayWithMask1: StoryObj<ImageSegmentationOverlayStoryArgs> = {
  render: ImageSegmentationOverlayStoryComponent,
  args: {
    segmentationMaskTopic: MASK_TOPIC_1,
    segmentationMaskOpacity: 0.5,
  },
};

export const OpacityControlWithMask1: StoryObj<ImageSegmentationOverlayStoryArgs> = {
  render: ImageSegmentationOverlayStoryComponent,
  args: {
    segmentationMaskTopic: MASK_TOPIC_1,
    segmentationMaskOpacity: 0.7,
  },
  argTypes: {
    segmentationMaskOpacity: { control: { type: "range", min: 0, max: 1, step: 0.05 } },
  },
};

export const BasicOverlayWithMask2: StoryObj<ImageSegmentationOverlayStoryArgs> = {
  render: ImageSegmentationOverlayStoryComponent,
  args: {
    segmentationMaskTopic: MASK_TOPIC_2,
    segmentationMaskOpacity: 0.6,
  },
};

export const FullOpacityMask1: StoryObj<ImageSegmentationOverlayStoryArgs> = {
  render: ImageSegmentationOverlayStoryComponent,
  args: {
    segmentationMaskTopic: MASK_TOPIC_1,
    segmentationMaskOpacity: 1.0,
  },
};

export const ZeroOpacityMask1: StoryObj<ImageSegmentationOverlayStoryArgs> = {
  render: ImageSegmentationOverlayStoryComponent,
  args: {
    segmentationMaskTopic: MASK_TOPIC_1,
    segmentationMaskOpacity: 0.0,
  },
};

export const NoMaskSelected: StoryObj<ImageSegmentationOverlayStoryArgs> = {
  render: ImageSegmentationOverlayStoryComponent,
  args: {
    segmentationMaskTopic: undefined, // Explicitly undefined
    segmentationMaskOpacity: 0.5, // Opacity is irrelevant if no mask topic
  },
};

export const MaskTopicNotAvailable: StoryObj<ImageSegmentationOverlayStoryArgs> = {
  render: ImageSegmentationOverlayStoryComponent,
  args: {
    segmentationMaskTopic: MASK_TOPIC_NONEXISTENT, // This topic has no messages in the fixture
    segmentationMaskOpacity: 0.5,
  },
};

export const MainImageNotVisible: StoryObj<ImageSegmentationOverlayStoryArgs> = {
  render: ImageSegmentationOverlayStoryComponent,
  args: {
    mainImageVisible: false,
    segmentationMaskTopic: MASK_TOPIC_1,
    segmentationMaskOpacity: 0.5,
  },
};
