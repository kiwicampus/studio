// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import * as THREE from "three";

import { AnyImage } from "./ImageTypes";
import { IRenderer } from "../../IRenderer";
import { BaseUserData, Renderable } from "../../Renderable";
import { SRGBToLinear } from "../../color";

const tempColor = new THREE.Color();

function hexToRgb(hex: string): { r: number; g: number; b: number } {
  const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
  if (!result) {
    return { r: 0, g: 0, b: 0 };
  }
  const [, r, g, b] = result;
  if (!r || !g || !b) {
    return { r: 0, g: 0, b: 0 };
  }
  return {
    r: parseInt(r, 16) / 255,
    g: parseInt(g, 16) / 255,
    b: parseInt(b, 16) / 255,
  };
}

function rgbToHex(color: THREE.Color): string {
  return `#${color.getHexString()}`;
}

export type SegmentationMaskUserData = BaseUserData & {
  topic: string;
  settings: {
    alpha: number;
    useRandomColors: boolean;
    colorMap: Record<number, string>;
  };
  image?: AnyImage;
  texture?: THREE.Texture;
  material?: THREE.Material;
  geometry?: THREE.PlaneGeometry;
  mesh?: THREE.Mesh;
  messageTime: bigint;
  firstMessageTime: bigint;
};

interface TextureWithData extends THREE.Texture {
  image: { data: Uint8ClampedArray | Uint8Array };
}

export class SegmentationMaskRenderable extends Renderable<SegmentationMaskUserData> {
  #colorCache = new Map<number, string>();

  public constructor(topicName: string, renderer: IRenderer, userData: SegmentationMaskUserData) {
    super(topicName, renderer, userData);
    this.renderOrder = 1;
    this.matrixAutoUpdate = true;
    this.matrix.identity();
  }

  public override dispose(): void {
    this.userData.texture?.dispose();
    this.userData.material?.dispose();
    this.userData.geometry?.dispose();
    super.dispose();
  }

  public setImage(image: AnyImage): void {
    console.log("SegmentationMaskRenderable.setImage called", image);
    this.userData.image = image;
    this.#updateTexture();
  }

  #getRandomColor(classId: number): string {
    let color = this.#colorCache.get(classId);
    if (!color) {
      // Generate a random color with good contrast
      tempColor.setHSL(Math.random(), 0.75, 0.65);
      color = rgbToHex(tempColor);
      this.#colorCache.set(classId, color);
    }
    return color;
  }

  #getColorForClass(classId: number): string {
    console.log("Getting color for class", classId);
    if (this.userData.settings.useRandomColors) {
      const color = this.#getRandomColor(classId);
      console.log("Random color", color);
      return color;
    }
    const colorMap = this.userData.settings.colorMap;
    return colorMap[classId] ?? "#000000";
  }

  #updateTexture(): void {
    console.log("SegmentationMaskRenderable.#updateTexture called");
    const image = this.userData.image;
    if (!image || !("data" in image)) {
      console.log("Invalid image format", image);
      return;
    }

    // Handle compressed image
    if ("format" in image && image.format === "png") {
      console.log("Handling compressed PNG image");
      // Create a blob from the compressed data
      const blob = new Blob([image.data], { type: "image/png" });
      // Create an image bitmap from the blob
      createImageBitmap(blob).then((bitmap) => {
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
      console.log("Missing width/height in raw image", image);
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
      console.log("Invalid image data types", { width, height, imageDataType: typeof imageData });
      return;
    }

    this.#processImageData(imageData, width, height);
  }

  // Helper method to process the image data once we have it in raw form
  #processImageData(
    imageData: Uint8Array | Uint8ClampedArray,
    width: number,
    height: number,
  ): void {
    // Create texture if it doesn't exist
    if (!this.userData.texture) {
      this.userData.texture = new THREE.DataTexture(
        new Uint8Array(width * height * 4),
        width,
        height,
        THREE.RGBAFormat,
      ) as TextureWithData;
      this.userData.texture.minFilter = THREE.LinearFilter;
      this.userData.texture.magFilter = THREE.LinearFilter;
    }

    const texture = this.userData.texture as TextureWithData;
    const data = new Uint8Array(width * height * 4);
    const alpha = Math.round(this.userData.settings.alpha * 255);

    // For compressed PNG images, data comes in RGBA format (4 bytes per pixel)
    const isPNG = imageData.length === width * height * 4;
    const stride = isPNG ? 4 : 1;

    // First pass: collect unique class IDs
    const uniqueClassIds = new Set<number>();
    // Let's see what class IDs we're getting
    const counts = new Map<number, number>();
    for (let i = 0; i < width * height; i++) {
      uniqueClassIds.add(imageData[i * stride]!);
      const classId = imageData[i * stride]!;
      counts.set(classId, (counts.get(classId) ?? 0) + 1);
    }
    console.log("Class ID distribution:", Object.fromEntries(counts.entries()));

    // Pre-calculate colors for all unique class IDs
    const colorCache = new Map<number, { r: number; g: number; b: number }>();
    for (const classId of uniqueClassIds) {
      const colorHex = this.#getColorForClass(classId);
      colorCache.set(classId, hexToRgb(colorHex));
    }
    // For now hardcode color cache for 0 to 4 classes
    colorCache.set(0, { r: 0, g: 1, b: 0 });
    colorCache.set(1, { r: 1, g: 0, b: 0 });
    colorCache.set(2, { r: 0, g: 0, b: 1 });
    colorCache.set(3, { r: 1, g: 1, b: 0 });
    colorCache.set(4, { r: 0, g: 1, b: 1 });
    console.log("colorCache", colorCache);

    // Fill the texture data using cached colors
    const samplePixels = new Map<number, { r: number; g: number; b: number }>();

    for (let i = 0; i < width * height; i++) {
      const classId = imageData[i * stride]!;
      const rgb = colorCache.get(classId)!;
      const j = i * 4;
      // data[j] = SRGBToLinear(rgb.r) * 255;
      // data[j + 1] = SRGBToLinear(rgb.g) * 255;
      // data[j + 2] = SRGBToLinear(rgb.b) * 255;
      data[j] = Math.round(rgb.r * 255);
      data[j + 1] = Math.round(rgb.g * 255);
      data[j + 2] = Math.round(rgb.b * 255);
      data[j + 3] = alpha;

      // Store a sample of each class ID's color
      if (!samplePixels.has(classId)) {
        samplePixels.set(classId, {
          r: data[j]!,
          g: data[j + 1]!,
          b: data[j + 2]!,
        });
      }
    }
    // console.log("Sample pixel colors:", Object.fromEntries(samplePixels.entries()));

    // Fill with a test pattern - should see red/green checkerboard
    // for (let i = 0; i < width * height; i++) {
    //   const x = Math.floor(i % width);
    //   const section = Math.floor((x / width) * 4); // Divide into 4 vertical sections
    //   const j = i * 4;

    //   switch (section) {
    //     case 0: // Pure Red
    //       data[j] = SRGBToLinear(1) * 255;
    //       data[j + 1] = 0;
    //       data[j + 2] = 0;
    //       break;
    //     case 1: // Pure Green
    //       data[j] = 0;
    //       data[j + 1] = SRGBToLinear(1) * 255;
    //       data[j + 2] = 0;
    //       break;
    //     case 2: // Pure Blue
    //       data[j] = 0;
    //       data[j + 1] = 0;
    //       data[j + 2] = SRGBToLinear(1) * 255;
    //       break;
    //     case 3: // White
    //       data[j] = SRGBToLinear(1) * 255;
    //       data[j + 1] = 255;
    //       data[j + 2] = 255;
    //       break;
    //   }
    //   data[j + 3] = alpha; // Full alpha
    // }

    texture.image.data = data;
    texture.format = THREE.RGBAFormat;
    texture.type = THREE.UnsignedByteType;
    texture.needsUpdate = true;
    texture.flipY = true;

    // Save the texture data for debugging
    // this.saveTextureToFile(data, width, height);

    // Create material if it doesn't exist
    if (!this.userData.material) {
      this.userData.material = new THREE.MeshBasicMaterial({
        map: texture,
        transparent: true,
        opacity: 1.0,
        depthTest: false,
        depthWrite: false,
        side: THREE.DoubleSide,
        blending: THREE.NormalBlending,
      });
    }

    // Create geometry and mesh if they don't exist
    if (!this.userData.geometry) {
      // Create a unit quad (1x1) centered at origin
      this.userData.geometry = new THREE.PlaneGeometry(1, 1);
      this.userData.mesh = new THREE.Mesh(this.userData.geometry, this.userData.material);

      // Scale to match image dimensions
      this.userData.mesh.scale.set(width, height, 1);

      // Position at origin in front of everything
      this.userData.mesh.position.set(0, 0, 0.01);
      this.userData.mesh.renderOrder = 1000;

      // Add to scene
      this.add(this.userData.mesh);
    }

    // Update geometry if dimensions changed
    if (
      this.userData.mesh &&
      (this.userData.geometry.parameters.width !== width ||
        this.userData.geometry.parameters.height !== height)
    ) {
      // Just update the scale
      this.userData.mesh.scale.set(width, height, 1);
      this.userData.mesh.position.set(0, 0, 0.01);
    }

    // Ensure visibility
    if (this.userData.mesh) {
      this.userData.mesh.visible = true;
      this.visible = true;
    }

    console.log("Mesh world position:", this.userData.mesh?.getWorldPosition(new THREE.Vector3()));
    console.log("Parent world matrix:", this.parent?.matrixWorld);
    console.log("Mesh local matrix:", this.userData.mesh?.matrix);
    this.debugState();

    console.log("Texture debug:", {
      format: texture.format,
      type: texture.type,
      encoding: texture.encoding,
      flipY: texture.flipY,
      premultiplyAlpha: texture.premultiplyAlpha,
      colorSpace: texture.colorSpace,
      // Sample first few pixels
      firstPixels: Array.from(data.slice(0, 16)),
    });

    console.log("Transform debug:", {
      meshScale: this.userData.mesh?.scale.toArray(),
      meshPosition: this.userData.mesh?.position.toArray(),
      meshMatrix: Array.from(this.userData.mesh?.matrix.elements ?? []),
      parentMatrix: Array.from(this.parent?.matrix.elements ?? []),
      worldMatrix: Array.from(this.userData.mesh?.matrixWorld.elements ?? []),
      matrixAutoUpdate: this.userData.mesh?.matrixAutoUpdate,
    });

    console.log("Geometry debug:", {
      parameters: this.userData.geometry.parameters,
      boundingBox: this.userData.geometry.boundingBox,
      center: this.userData.geometry.boundingSphere?.center,
    });

    this.userData.mesh?.updateMatrixWorld(true); // Force update of world matrix
    this.updateMatrixWorld(true); // Force update of parent matrices
  }

  public updateSettings(settings: Partial<SegmentationMaskUserData["settings"]>): void {
    // Update the settings
    Object.assign(this.userData.settings, settings);

    // Update material opacity if it exists
    if (this.userData.material) {
      this.userData.material.opacity = this.userData.settings.alpha;
      this.userData.material.needsUpdate = true;
    }

    // Clear color cache and regenerate texture if color settings changed
    if (settings.useRandomColors !== undefined || settings.colorMap !== undefined) {
      this.#colorCache.clear();
      this.#updateTexture();
    } else if (settings.alpha !== undefined) {
      // If only alpha changed, just update the texture alpha values
      const texture = this.userData.texture as TextureWithData;
      if (texture) {
        const alpha = Math.round(this.userData.settings.alpha * 255);
        const data = texture.image.data;
        for (let i = 3; i < data.length; i += 4) {
          data[i] = alpha;
        }
        texture.needsUpdate = true;
      }
    }
  }

  // Add a method to check the mesh state
  public debugState(): void {
    if (this.userData.mesh) {
      console.log("Mask mesh state:", {
        position: this.userData.mesh.position,
        scale: this.userData.mesh.scale,
        visible: this.userData.mesh.visible,
        renderOrder: this.renderOrder,
        materialTransparent: this.userData.material?.transparent,
        materialOpacity: this.userData.material!.opacity,
        textureSize: {
          width: (this.userData.texture as THREE.DataTexture).image.width,
          height: (this.userData.texture as THREE.DataTexture).image.height,
        },
      });
    } else {
      console.log("No mask mesh created yet");
    }
  }

  // Add this helper method to save the texture data as PNG
  private saveTextureToFile(data: Uint8Array, width: number, height: number): void {
    // Create a canvas to draw the data
    const canvas = document.createElement("canvas");
    canvas.width = width;
    canvas.height = height;
    const ctx = canvas.getContext("2d");
    if (!ctx) {
      return;
    }

    // Create ImageData from our RGBA array
    const imageData = new ImageData(new Uint8ClampedArray(data), width, height);
    ctx.putImageData(imageData, 0, 0);

    // Convert to blob and save
    canvas.toBlob((blob) => {
      if (!blob) {
        return;
      }
      const url = URL.createObjectURL(blob);
      const link = document.createElement("a");
      link.href = url;
      link.download = "segmentation_mask_debug.png";
      link.click();
      URL.revokeObjectURL(url);
    }, "image/png");
  }
}
