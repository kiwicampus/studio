// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import * as THREE from "three";

import { AnyImage } from "./ImageTypes";
import { IRenderer } from "../../IRenderer";
import { BaseUserData, Renderable } from "../../Renderable";

const tempColor = new THREE.Color();

export type SegmentationMaskUserData = BaseUserData & {
  topic: string;
  settings: {
    alpha: number;
    useRandomColors: boolean;
    colorMap: Record<number, string>;
  };
  image?: AnyImage;
  messageTime: bigint;
  firstMessageTime: bigint;
};

export class SegmentationMaskRenderable extends Renderable<SegmentationMaskUserData> {
  #colorCache = new Map<number, string>();
  #imageRenderable?: THREE.Texture;

  public constructor(topicName: string, renderer: IRenderer, userData: SegmentationMaskUserData) {
    super(topicName, renderer, userData);
  }

  public override dispose(): void {
    super.dispose();
  }

  public setImage(image: AnyImage): void {
    this.userData.image = image;
    this.#updateTexture();
  }

  #getRandomColor(classId: number): string {
    let color = this.#colorCache.get(classId);
    if (!color) {
      // Generate a random color with good contrast
      tempColor.setHSL(Math.random(), 0.75, 0.65);
      color = `#${tempColor.getHexString()}`;
      this.#colorCache.set(classId, color);
    }
    return color;
  }

  #getColorForClass(classId: number): string {
    if (this.userData.settings.useRandomColors) {
      return this.#getRandomColor(classId);
    }
    const colorMap = this.userData.settings.colorMap;
    return colorMap[classId] ?? "#000000";
  }

  #processImageData(
    imageData: Uint8Array | Uint8ClampedArray,
    width: number,
    height: number,
  ): void {
    // Get the existing texture from ImageRenderable
    const texture = this.#imageRenderable;
    if (!texture) {
      console.error("No texture found");
      return;
    }

    // Create a canvas to blend the mask
    const canvas = document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext("2d");
    if (!ctx) {
      console.error("Failed to create canvas context");
      return;
    }

    // Draw the original image based on its type
    if (texture instanceof THREE.CanvasTexture) {
      if (!(texture.image instanceof HTMLCanvasElement || texture.image instanceof ImageBitmap)) {
        console.error("Invalid texture image", texture.image);
        return;
      }
      console.log("CanvasTexture", texture);
      ctx.drawImage(texture.image, 0, 0);
    } else if (texture instanceof THREE.DataTexture) {
      console.log("DataTexture", texture);
      // For DataTexture, we need to create an ImageData from the raw data
      const textureImageData = new ImageData(
        new Uint8ClampedArray(texture.image.data),
        texture.image.width,
        texture.image.height,
      );
      ctx.putImageData(textureImageData, 0, 0);
    } else {
      return;
    }

    const originalImageData = ctx.getImageData(0, 0, width, height);
    const originalData = originalImageData.data;

    // Create an ImageData for the mask
    const maskData = ctx.createImageData(width, height);
    const data = maskData.data;
    const alpha = Math.round(this.userData.settings.alpha * 255);

    // For compressed PNG images, data comes in RGBA format (4 bytes per pixel)
    const isPNG = imageData.length === width * height * 4;
    const stride = isPNG ? 4 : 1;

    // // First pass: collect unique class IDs
    // const uniqueClassIds = new Set<number>();
    // // Let's see what class IDs we're getting
    // const counts = new Map<number, number>();
    // for (let i = 0; i < width * height; i++) {
    //   uniqueClassIds.add(imageData[i * stride]!);
    //   const classId = imageData[i * stride]!;
    //   counts.set(classId, (counts.get(classId) ?? 0) + 1);
    // }
    // console.log("Class ID distribution:", Object.fromEntries(counts.entries()));

    // // Pre-calculate colors for all unique class IDs
    const colorCache = new Map<number, { r: number; g: number; b: number }>();
    // for (const classId of uniqueClassIds) {
    //   const colorHex = this.#getColorForClass(classId);
    //   colorCache.set(classId, hexToRgb(colorHex));
    // }
    // For now hardcode color cache for 0 to 4 classes
    colorCache.set(0, { r: 1, g: 0, b: 0 });
    colorCache.set(1, { r: 0, g: 1, b: 0 });
    colorCache.set(2, { r: 0, g: 0, b: 1 });
    colorCache.set(3, { r: 1, g: 1, b: 0 });
    colorCache.set(4, { r: 0, g: 1, b: 1 });
    colorCache.set(5, { r: 1, g: 0, b: 1 });
    colorCache.set(6, { r: 1, g: 1, b: 1 });

    // Fill the mask data and blend with original
    for (let i = 0; i < width * height; i++) {
      const classId = imageData[i * stride]!;
      const rgb = colorCache.get(classId)!;
      // check rgb is not undefined
      if (rgb == undefined) {
        console.error("No color found for class ID", classId);
        continue;
      }
      const j = i * 4;

      // Get original pixel values
      const origR = originalData[j] ?? 0;
      const origG = originalData[j + 1] ?? 0;
      const origB = originalData[j + 2] ?? 0;

      // Calculate blended colors
      const maskAlpha = alpha / 255;
      data[j] = Math.round(origR * (1 - maskAlpha) + rgb.r * 255 * maskAlpha);
      data[j + 1] = Math.round(origG * (1 - maskAlpha) + rgb.g * 255 * maskAlpha);
      data[j + 2] = Math.round(origB * (1 - maskAlpha) + rgb.b * 255 * maskAlpha);
      data[j + 3] = 255; // Keep fully opaque
    }

    // Clear the canvas and draw the final blended result
    ctx.clearRect(0, 0, width, height);
    ctx.putImageData(maskData, 0, 0);

    // Now flip the final result for Three.js texture coordinates
    const finalCanvas = document.createElement("canvas");
    finalCanvas.width = width;
    finalCanvas.height = height;
    const finalCtx = finalCanvas.getContext("2d")!;
    finalCtx.translate(0, height);
    finalCtx.scale(1, -1);
    finalCtx.drawImage(canvas, 0, 0);

    // Update the texture with the flipped result
    texture.image = finalCanvas;
    texture.needsUpdate = true;
  }

  public updateSettings(settings: Partial<SegmentationMaskUserData["settings"]>): void {
    Object.assign(this.userData.settings, settings);

    // If color settings changed, update the texture
    if (settings.useRandomColors != undefined || settings.colorMap != undefined) {
      this.#colorCache.clear();
      if (this.userData.image) {
        this.#updateTexture();
      }
    } else if (settings.alpha != undefined) {
      this.#updateTexture();
    }
  }

  #updateTexture(): void {
    const image = this.userData.image;
    if (!image || !("data" in image)) {
      return;
    }

    // Handle compressed image
    if ("format" in image && image.format === "png") {
      // Create a blob from the compressed data
      const blob = new Blob([image.data], { type: "image/png" });
      // Create an image bitmap from the blob
      void createImageBitmap(blob).then((bitmap) => {
        // Create a canvas to read the pixel data
        const canvas = document.createElement("canvas");
        canvas.width = bitmap.width;
        canvas.height = bitmap.height;
        const ctx = canvas.getContext("2d");
        if (!ctx) {
          return;
        }

        // Draw the bitmap to get pixel data
        ctx.drawImage(bitmap, 0, 0);
        const imageData = ctx.getImageData(0, 0, bitmap.width, bitmap.height);

        // Now process the decompressed data
        this.#processImageData(imageData.data, bitmap.width, bitmap.height);
      });
      return;
    }

    // Handle raw image
    if (!("width" in image) || !("height" in image)) {
      return;
    }

    const width = image.width;
    const height = image.height;
    const imageData = image.data;

    if (
      typeof width !== "number" ||
      typeof height !== "number" ||
      !(imageData instanceof Uint8Array)
    ) {
      return;
    }

    this.#processImageData(imageData, width, height);
  }

  public update(): void {
    if (this.userData.image) {
      this.#updateTexture();
    }
  }

  public setImageRenderable(texture: THREE.Texture): void {
    this.#imageRenderable = texture;
    if (this.userData.image) {
      this.#updateTexture();
    }
  }
}
