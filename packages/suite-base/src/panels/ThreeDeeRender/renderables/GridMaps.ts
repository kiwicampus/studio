// SPDX-FileCopyrightText: Copyright (C) 2023-2025 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import { t } from "i18next";
import * as THREE from "three";

import { toNanoSec } from "@lichtblick/rostime";
import { SettingsTreeAction, SettingsTreeFields } from "@lichtblick/suite";
import type { RosValue } from "@lichtblick/suite-base/players/types";

import type { AnyRendererSubscription, IRenderer } from "../IRenderer";
import { BaseUserData, Renderable } from "../Renderable";
import {
  PartialMessage,
  PartialMessageEvent,
  SceneExtension,
  onlyLastByTopicMessage,
} from "../SceneExtension";
import { SettingsTreeEntry } from "../SettingsManager";
import { rgbaToCssString, stringToRgba } from "../color";
import { normalizeHeader, normalizePose } from "../normalizeMessages";
import { ColorRGBA, GridMap, Float32MultiArray, GRID_MAP_DATATYPES } from "../ros";
import { BaseSettings } from "../settings";
import { topicIsConvertibleToSchema } from "../topicIsConvertibleToSchema";

export type LayerSettingsGridMap = BaseSettings & {
  heightLayer: string;
  colorLayer: string;
  colorMode: "gradient" | "rainbow";
  minColor: string;
  maxColor: string;
  minValue: number;
  maxValue: number;
  useAutoMinMax: boolean;
  alpha: number;
  flatTerrain: boolean;
  showGridLines: boolean;
};

const INVALID_GRID_MAP = "INVALID_GRID_MAP";

/** Set to true and check browser console to debug GridMap coordinate issues */
const DEBUG_GRIDMAP = false;

/** Fallback when frame_id is empty. Try odom first (elevation maps often use odom), then map */
const EMPTY_FRAME_ID_FALLBACK = "odom";

const DEFAULT_MIN_COLOR = { r: 0, g: 0, b: 1, a: 1 };
const DEFAULT_MAX_COLOR = { r: 1, g: 0, b: 0, a: 1 };

const DEFAULT_MIN_COLOR_STR = rgbaToCssString(DEFAULT_MIN_COLOR);
const DEFAULT_MAX_COLOR_STR = rgbaToCssString(DEFAULT_MAX_COLOR);
const DEFAULT_ALPHA = 0.9;

const DEFAULT_SETTINGS: LayerSettingsGridMap = {
  visible: true,
  heightLayer: "elevation",
  colorLayer: "elevation",
  colorMode: "gradient",
  minColor: DEFAULT_MIN_COLOR_STR,
  maxColor: DEFAULT_MAX_COLOR_STR,
  minValue: 0,
  maxValue: 1,
  useAutoMinMax: true,
  alpha: DEFAULT_ALPHA,
  flatTerrain: false,
  showGridLines: true,
};

export type GridMapUserData = BaseUserData & {
  settings: LayerSettingsGridMap;
  topic: string;
  gridMap: GridMap;
  mesh: THREE.Mesh;
  material: THREE.MeshBasicMaterial;
  gridLines: THREE.LineSegments;
  gridLineMaterial: THREE.LineBasicMaterial;
};

export class GridMapRenderable extends Renderable<GridMapUserData> {
  public override dispose(): void {
    this.userData.mesh.geometry.dispose();
    this.userData.material.dispose();
    this.userData.gridLines.geometry.dispose();
    this.userData.gridLineMaterial.dispose();
  }

  public override details(): Record<string, RosValue> {
    return this.userData.gridMap;
  }
}

/**
 * Accessor over a single GridMap layer's Float32MultiArray, indexed by logical
 * (rowIdx, colIdx) in [0,rows)x[0,cols).
 *
 * grid_map_msgs does not always store layer data row-major: whichever of
 * dim[0]/dim[1] is labeled "row_index" vs "column_index" tells us which axis
 * is which, but per the std_msgs/MultiArrayLayout spec the flat offset is
 * always `data_offset + d0 * dim[1].size + d1`, where d0/d1 index dim[0]/dim[1]
 * in message order (not necessarily row/col order). grid_map_ros commonly
 * publishes dim[0]="column_index" (outer/slow) and dim[1]="row_index"
 * (inner/fast) because it maps the layer's column-major Eigen matrix directly
 * onto the message buffer, so treating the data as row-major (as if dim[0]
 * were always rows) silently transposes and aliases the grid.
 *
 * Each layer is also a circular buffer. Despite the field names, GridMap.msg
 * defines outer_start_index as the row start and inner_start_index as the
 * column start (Index(row, col) in grid_map_core) — not the MultiArray
 * dim[0]/dim[1] starts. Wrap into buffer indices first, then map to d0/d1.
 */
function createLayerAccessor(
  array: Float32MultiArray,
  outerStartIndex: number,
  innerStartIndex: number,
): { rows: number; cols: number; get: (rowIdx: number, colIdx: number) => number } {
  const layout = array.layout;
  const dim0 = layout?.dim?.[0];
  const dim1 = layout?.dim?.[1];
  if (!dim0 || !dim1) {
    return { rows: 0, cols: 0, get: () => NaN };
  }

  const dim0IsRow = dim0.label === "row_index";
  const rows = dim0IsRow ? dim0.size : dim1.size;
  const cols = dim0IsRow ? dim1.size : dim0.size;
  const dataOffset = layout.data_offset ?? 0;
  const data = array.data instanceof Float32Array ? array.data : new Float32Array(array.data);

  return {
    rows,
    cols,
    get(rowIdx: number, colIdx: number): number {
      // outer_start_index = row start, inner_start_index = column start
      const rowBuf = (rowIdx + outerStartIndex) % rows;
      const colBuf = (colIdx + innerStartIndex) % cols;
      const d0 = dim0IsRow ? rowBuf : colBuf;
      const d1 = dim0IsRow ? colBuf : rowBuf;
      return data[dataOffset + d0 * dim1.size + d1] ?? NaN;
    },
  };
}

/** Converts HSV to RGB (h in [0,360), s and v in [0,1]) */
function hsvToRgb(h: number, s: number, v: number): { r: number; g: number; b: number } {
  h = ((h % 360) + 360) % 360;
  const c = v * s;
  const x = c * (1 - Math.abs(((h / 60) % 2) - 1));
  const m = v - c;
  let r = 0,
    g = 0,
    b = 0;
  if (h < 60) {
    r = c;
    g = x;
    b = 0;
  } else if (h < 120) {
    r = x;
    g = c;
    b = 0;
  } else if (h < 180) {
    r = 0;
    g = c;
    b = x;
  } else if (h < 240) {
    r = 0;
    g = x;
    b = c;
  } else if (h < 300) {
    r = x;
    g = 0;
    b = c;
  } else {
    r = c;
    g = 0;
    b = x;
  }
  return { r: r + m, g: g + m, b: b + m };
}

export class GridMaps extends SceneExtension<GridMapRenderable> {
  public static extensionId = "foxglove.GridMaps";

  public constructor(renderer: IRenderer, name: string = GridMaps.extensionId) {
    super(name, renderer);
  }

  public override getSubscriptions(): readonly AnyRendererSubscription[] {
    return [
      {
        type: "schema",
        schemaNames: GRID_MAP_DATATYPES,
        subscription: { handler: this.#handleGridMap, filterQueue: onlyLastByTopicMessage },
      },
    ];
  }

  public override settingsNodes(): SettingsTreeEntry[] {
    const configTopics = this.renderer.config.topics;
    const handler = this.handleSettingsAction;
    const entries: SettingsTreeEntry[] = [];
    for (const topic of this.renderer.topics ?? []) {
      if (!topicIsConvertibleToSchema(topic, GRID_MAP_DATATYPES)) {
        continue;
      }

      const configWithDefaults = { ...DEFAULT_SETTINGS, ...configTopics[topic.name] };

      const layerOptions = this.#getLayerOptions(configTopics[topic.name]);
      const heightLayerOptions = layerOptions;
      const colorLayerOptions = [{ label: t("threeDee:flatColor"), value: "__flat__" }, ...layerOptions];

      const fields: SettingsTreeFields = {
        heightLayer: {
          label: t("threeDee:heightLayer"),
          input: "select",
          value: configWithDefaults.heightLayer,
          options: heightLayerOptions,
        },
        colorLayer: {
          label: t("threeDee:colorLayer"),
          input: "select",
          value: configWithDefaults.colorLayer,
          options: colorLayerOptions,
        },
        colorMode: {
          label: t("threeDee:colorMode"),
          input: "select",
          value: configWithDefaults.colorMode,
          options: [
            { label: t("threeDee:gridMapColorModeGradient"), value: "gradient" },
            { label: t("threeDee:gridMapColorModeRainbow"), value: "rainbow" },
          ],
        },
        flatTerrain: {
          label: t("threeDee:flatTerrain"),
          input: "boolean",
          value: configWithDefaults.flatTerrain,
        },
        minColor: {
          label: t("threeDee:minColor"),
          input: "rgba",
          value: configWithDefaults.minColor,
        },
        maxColor: {
          label: t("threeDee:maxColor"),
          input: "rgba",
          value: configWithDefaults.maxColor,
        },
        useAutoMinMax: {
          label: t("threeDee:gridMapAutoMinMax"),
          input: "boolean",
          value: configWithDefaults.useAutoMinMax,
        },
        minValue: {
          label: t("threeDee:gridMapMinValue"),
          input: "number",
          value: configWithDefaults.minValue,
        },
        maxValue: {
          label: t("threeDee:gridMapMaxValue"),
          input: "number",
          value: configWithDefaults.maxValue,
        },
        showGridLines: {
          label: t("threeDee:gridMapShowGridLines"),
          input: "boolean",
          value: configWithDefaults.showGridLines,
        },
        alpha: {
          label: "Alpha",
          input: "number",
          value: configWithDefaults.alpha,
          min: 0,
          max: 1,
          step: 0.1,
        },
      };

      entries.push({
        path: ["topics", topic.name],
        node: {
          label: topic.name,
          icon: "Cells",
          fields,
          visible: configWithDefaults.visible,
          order: topic.name.toLocaleLowerCase(),
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
    const [_, topicName, field] = path;
    if (typeof topicName !== "string" || typeof field !== "string") {
      return;
    }
    this.saveSetting(path as ["topics", string, string], action.payload.value);
    const renderable = this.renderables.get(topicName) as GridMapRenderable | undefined;
    if (renderable) {
      const topicConfig = this.renderer.config.topics[topicName] as
        | Partial<LayerSettingsGridMap>
        | undefined;
      renderable.userData.settings = { ...DEFAULT_SETTINGS, ...topicConfig };

      const gridMap = this.#gridMap.get(topicName);
      if (gridMap) {
        this.#updateGridMapRenderable(renderable, gridMap, renderable.userData.receiveTime);
      }
    }
  };

  #gridMap = new Map<string, GridMap>();

  #getLayerOptions(topicConfig: Partial<LayerSettingsGridMap> | undefined): { label: string; value: string }[] {
    const layers = topicConfig?.heightLayer ? [topicConfig.heightLayer] : ["elevation"];
    const layerSet = new Set(layers);
    for (const gridMap of this.#gridMap.values()) {
      for (const layer of gridMap.layers ?? []) {
        layerSet.add(layer);
      }
    }
    return Array.from(layerSet).map((layer) => ({ label: layer, value: layer }));
  }

  #getEffectiveFrameId(messageFrameId: string): string {
    return (messageFrameId ?? "").trim() || EMPTY_FRAME_ID_FALLBACK;
  }

  #handleGridMap = (messageEvent: PartialMessageEvent<GridMap>): void => {
    const topic = messageEvent.topic;
    const gridMap = normalizeGridMap(messageEvent.message);
    const receiveTime = toNanoSec(messageEvent.receiveTime);

    const frameId = this.#getEffectiveFrameId(gridMap.info.header.frame_id ?? "");
    this.renderer.addCoordinateFrame(frameId);

    this.#gridMap.set(topic, gridMap);

    let renderable = this.renderables.get(topic) as GridMapRenderable | undefined;
    if (!renderable) {
      const configWithDefaults = {
        ...DEFAULT_SETTINGS,
        ...(this.renderer.config.topics[topic] as Partial<LayerSettingsGridMap> | undefined),
      };
      const material = new THREE.MeshBasicMaterial({
        vertexColors: true,
        side: THREE.DoubleSide,
      });
      const mesh = new THREE.Mesh(new THREE.BufferGeometry(), material);
      const gridLineMaterial = new THREE.LineBasicMaterial({
        color: 0x000000,
        transparent: true,
        opacity: 0.5,
      });
      const gridLines = new THREE.LineSegments(new THREE.BufferGeometry(), gridLineMaterial);
      // Use frame origin (like RViz): transform places frame origin; mesh vertices are absolute in frame
      const frameOriginPose = {
        position: { x: 0, y: 0, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      };
      renderable = new GridMapRenderable(topic, this.renderer, {
        receiveTime,
        messageTime: toNanoSec(gridMap.info.header.stamp),
        frameId: this.renderer.normalizeFrameId(frameId),
        pose: frameOriginPose,
        settingsPath: ["topics", topic],
        settings: configWithDefaults,
        topic,
        gridMap,
        mesh,
        material,
        gridLines,
        gridLineMaterial,
      });
      renderable.add(mesh);
      renderable.add(gridLines);
      this.add(renderable);
      this.renderables.set(topic, renderable);
    }

    this.#updateGridMapRenderable(renderable, gridMap, receiveTime);
  };

  #updateGridMapRenderable(
    renderable: GridMapRenderable,
    gridMap: GridMap,
    receiveTime: bigint,
  ): void {
    renderable.userData.gridMap = gridMap;
    renderable.userData.pose = {
      position: { x: 0, y: 0, z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    };
    renderable.userData.receiveTime = receiveTime;
    renderable.userData.messageTime = toNanoSec(gridMap.info.header.stamp);
    const effectiveFrameId = this.#getEffectiveFrameId(gridMap.info.header.frame_id ?? "");
    renderable.userData.frameId = this.renderer.normalizeFrameId(effectiveFrameId);

    const { layers, data, info } = gridMap;
    if (!layers?.length || !data?.length || layers.length !== data.length) {
      this.renderer.settings.errors.addToTopic(
        renderable.userData.topic,
        INVALID_GRID_MAP,
        "GridMap layers and data arrays must have matching length",
      );
      return;
    }

    const settings = renderable.userData.settings;
    const heightLayerIdx = layers.indexOf(settings.flatTerrain ? layers[0]! : settings.heightLayer);
    const colorLayerIdx = settings.colorLayer === "__flat__"
      ? -1
      : layers.indexOf(settings.colorLayer);

    if (heightLayerIdx < 0 && !settings.flatTerrain) {
      this.renderer.settings.errors.addToTopic(
        renderable.userData.topic,
        INVALID_GRID_MAP,
        `Height layer "${settings.heightLayer}" not found. Available: ${layers.join(", ")}`,
      );
      return;
    }

    const heightData = heightLayerIdx >= 0 ? data[heightLayerIdx]! : data[0]!;
    const colorData = colorLayerIdx >= 0 ? data[colorLayerIdx]! : heightData;

    const outerStartIndex = gridMap.outer_start_index;
    const innerStartIndex = gridMap.inner_start_index;
    const heightAccessor = createLayerAccessor(heightData, outerStartIndex, innerStartIndex);
    const colorAccessor = createLayerAccessor(colorData, outerStartIndex, innerStartIndex);
    const { rows, cols } = heightAccessor;
    if (rows < 2 || cols < 2) {
      this.renderer.settings.errors.addToTopic(
        renderable.userData.topic,
        INVALID_GRID_MAP,
        `GridMap must have at least 2x2 cells, got ${rows}x${cols}`,
      );
      return;
    }

    const resolution = info.resolution ?? 0.1;
    // grid_map convention: rows run along x (forward), cols run along y (left).
    const lengthX = info.length_x ?? resolution * rows;
    const lengthY = info.length_y ?? resolution * cols;
    const pose = info.pose;
    const centerX = pose.position.x ?? 0;
    const centerY = pose.position.y ?? 0;

    // Grid map center is at (centerX, centerY). Cell (0,0) is at top-left.
    // Per grid_map_core / RViz: renderable at frame origin; mesh vertices at absolute positions in frame
    const topLeftX = centerX + lengthX / 2 - resolution / 2;
    const topLeftY = centerY + lengthY / 2 - resolution / 2;

    if (DEBUG_GRIDMAP) {
      const frameId = this.#getEffectiveFrameId(gridMap.info.header.frame_id ?? "");
      console.log("[GridMap]", {
        topic: renderable.userData.topic,
        frameId,
        center: { x: centerX, y: centerY },
        lengthX,
        lengthY,
        resolution,
        rows,
        cols,
        topLeft: { x: topLeftX, y: topLeftY },
      });
    }

    // Compute isValid mask from basic_layers (RViz: skip invalid cells to create holes)
    const gridMapData = gridMap.data;
    const basicLayers = gridMap.basic_layers ?? [];
    const validityLayers = basicLayers.includes(settings.heightLayer)
      ? basicLayers
      : [settings.heightLayer, ...basicLayers];
    const validityAccessors = validityLayers
      .map((layerName) => {
        const layerIdx = layers.indexOf(layerName);
        if (layerIdx < 0 || !gridMapData[layerIdx]) {
          return undefined;
        }
        return createLayerAccessor(gridMapData[layerIdx]!, outerStartIndex, innerStartIndex);
      })
      .filter((accessor): accessor is ReturnType<typeof createLayerAccessor> => accessor != undefined);
    const isValidCell = (i: number, j: number): boolean => {
      for (const accessor of validityAccessors) {
        if (!Number.isFinite(accessor.get(i, j))) {
          return false;
        }
      }
      return true;
    };

    const geometry = new THREE.BufferGeometry();
    const positions: number[] = [];
    const colors: number[] = [];
    const indices: number[] = [];

    const tempMin = { r: 0, g: 0, b: 0, a: 1 };
    const tempMax = { r: 0, g: 0, b: 0, a: 1 };
    stringToRgba(tempMin, settings.minColor);
    stringToRgba(tempMax, settings.maxColor);

    let minVal: number;
    let maxVal: number;
    if (settings.useAutoMinMax && settings.colorLayer !== "__flat__") {
      minVal = Number.POSITIVE_INFINITY;
      maxVal = Number.NEGATIVE_INFINITY;
      for (let i = 0; i < rows; i++) {
        for (let j = 0; j < cols; j++) {
          const v = colorAccessor.get(i, j);
          if (Number.isFinite(v)) {
            minVal = Math.min(minVal, v);
            maxVal = Math.max(maxVal, v);
          }
        }
      }
      if (minVal === maxVal) maxVal = minVal + 1;
    } else {
      minVal = settings.minValue;
      maxVal = settings.maxValue;
      if (minVal === maxVal) maxVal = minVal + 1;
    }

    const getColor = (value: number): ColorRGBA => {
      if (settings.colorLayer === "__flat__" || !Number.isFinite(value)) {
        return { r: 0.5, g: 0.5, b: 0.5, a: settings.alpha };
      }
      const t = (value - minVal) / (maxVal - minVal);
      const clamped = Math.max(0, Math.min(1, t));
      if (settings.colorMode === "rainbow") {
        const { r, g, b } = hsvToRgb(clamped * 300, 1, 1);
        return { r, g, b, a: settings.alpha };
      }
      return {
        r: tempMin.r + (tempMax.r - tempMin.r) * clamped,
        g: tempMin.g + (tempMax.g - tempMin.g) * clamped,
        b: tempMin.b + (tempMax.b - tempMin.b) * clamped,
        a: settings.alpha,
      };
    };

    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < cols; j++) {
        const rawH = heightAccessor.get(i, j);
        const valid = isValidCell(i, j);
        // Invalid cells: skip triangle to create hole; use 0 for vertex position
        const h: number = settings.flatTerrain ? 0 : (Number.isFinite(rawH) ? rawH : 0);

        const x = topLeftX - i * resolution;
        const y = topLeftY - j * resolution;

        positions.push(x, y, h);

        const colorVal = colorAccessor.get(i, j);
        const color = getColor(typeof colorVal === "number" ? colorVal : 0);
        colors.push(color.r, color.g, color.b, valid ? color.a : 0);

        if (i > 0 && j > 0) {
          const a = (i - 1) * cols + (j - 1);
          const b = (i - 1) * cols + j;
          const c = i * cols + (j - 1);
          const d = i * cols + j;
          const quadValid = valid && isValidCell(i - 1, j - 1) && isValidCell(i - 1, j) && isValidCell(i, j - 1);
          if (quadValid) {
            indices.push(a, c, d, a, d, b);
          }
        }
      }
    }

    geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    geometry.setAttribute("color", new THREE.Float32BufferAttribute(colors, 4));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();

    if (renderable.userData.mesh.geometry) {
      renderable.userData.mesh.geometry.dispose();
    }
    renderable.userData.mesh.geometry = geometry;
    renderable.userData.mesh.material = renderable.userData.material;

    renderable.userData.material.vertexColors = true;
    renderable.userData.material.transparent = settings.alpha < 1;
    renderable.userData.material.depthWrite = !renderable.userData.material.transparent;
    renderable.userData.material.side = THREE.DoubleSide;

    // Grid lines (RViz-style) - draw cell edges only where both adjacent cells have map data
    renderable.userData.gridLines.visible = settings.showGridLines;
    if (settings.showGridLines) {
      const getHeight = (i: number, j: number): number => {
        if (i < 0 || i >= rows || j < 0 || j >= cols) return 0;
        const v = heightAccessor.get(i, j);
        return Number.isFinite(v) ? v : 0;
      };
      const linePositions: number[] = [];
      for (let i = 0; i < rows; i++) {
        for (let j = 0; j < cols; j++) {
          const x = topLeftX - i * resolution;
          const y = topLeftY - j * resolution;
          const h = getHeight(i, j);
          if (j < cols - 1 && isValidCell(i, j) && isValidCell(i, j + 1)) {
            const y2 = topLeftY - (j + 1) * resolution;
            const h2 = getHeight(i, j + 1);
            linePositions.push(x, y, h, x, y2, h2);
          }
          if (i < rows - 1 && isValidCell(i, j) && isValidCell(i + 1, j)) {
            const x2 = topLeftX - (i + 1) * resolution;
            const h2 = getHeight(i + 1, j);
            linePositions.push(x, y, h, x2, y, h2);
          }
        }
      }
      const lineGeometry = new THREE.BufferGeometry();
      lineGeometry.setAttribute("position", new THREE.Float32BufferAttribute(linePositions, 3));
      lineGeometry.computeBoundingSphere();
      if (renderable.userData.gridLines.geometry) {
        renderable.userData.gridLines.geometry.dispose();
      }
      renderable.userData.gridLines.geometry = lineGeometry;
    }
    renderable.userData.gridLineMaterial.opacity = settings.alpha;

    this.renderer.settings.errors.remove(renderable.userData.settingsPath, INVALID_GRID_MAP);
  }
}

function normalizeGridMap(message: PartialMessage<GridMap>): GridMap {
  const info = message.info ?? {};
  const header = info.header ?? {};

  return {
    info: {
      header: normalizeHeader(header),
      resolution: info.resolution ?? 0.1,
      length_x: info.length_x ?? 1,
      length_y: info.length_y ?? 1,
      pose: normalizePose(info.pose),
    },
    layers: (message.layers ?? []).filter((l): l is string => typeof l === "string"),
    basic_layers: (message.basic_layers ?? []).filter((l): l is string => typeof l === "string"),
    data: (message.data ?? []).map((d) => {
      const layout = d?.layout;
      const dims = layout?.dim ?? [];
      const dim0 = dims[0];
      const dim1 = dims[1];
      return {
        layout: {
          dim: [
            { label: dim0?.label ?? "", size: dim0?.size ?? 0, stride: dim0?.stride ?? 0 },
            { label: dim1?.label ?? "", size: dim1?.size ?? 0, stride: dim1?.stride ?? 0 },
          ],
          data_offset: layout?.data_offset ?? 0,
        },
        data:
          d?.data instanceof Float32Array
            ? d.data
            : new Float32Array(Array.isArray(d?.data) ? (d.data as number[]) : []),
      };
    }),
    outer_start_index: message.outer_start_index ?? 0,
    inner_start_index: message.inner_start_index ?? 0,
  };
}
