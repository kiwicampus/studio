// SPDX-FileCopyrightText: Copyright (C) 2023-2025 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import { StoryObj } from "@storybook/react";

import { MessageEvent } from "@lichtblick/suite";
import { Topic } from "@lichtblick/suite-base/players/types";
import PanelSetup from "@lichtblick/suite-base/stories/PanelSetup";

import { QUAT_IDENTITY } from "./common";
import useDelayedFixture from "./useDelayedFixture";
import ThreeDeePanel from "../index";
import { GridMap, TransformStamped } from "../ros";

export default {
  title: "panels/ThreeDeeRender/GridMap",
  component: ThreeDeePanel,
  parameters: { colorScheme: "light" },
};

// Mirrors the real grid_map_ros wire format: layer data is the layer's Eigen
// matrix (rows x cols) copied directly into the buffer in column-major order,
// so dim[0] is "column_index" (outer/slow) and dim[1] is "row_index"
// (inner/fast) with flat index = colBuf * rows + rowBuf.
// outer_start_index / inner_start_index are logical row/col circular-buffer
// starts (GridMap.msg), so generator(i,j) is stored at the wrapped buffer index.
function createGridMapLayer(
  rows: number,
  cols: number,
  generator: (i: number, j: number) => number,
  outerStartIndex = 0,
  innerStartIndex = 0,
): { layout: { dim: { label: string; size: number; stride: number }[]; data_offset: number }; data: Float32Array } {
  const data = new Float32Array(rows * cols);
  for (let i = 0; i < rows; i++) {
    for (let j = 0; j < cols; j++) {
      const rowBuf = (i + outerStartIndex) % rows;
      const colBuf = (j + innerStartIndex) % cols;
      data[colBuf * rows + rowBuf] = generator(i, j);
    }
  }
  return {
    layout: {
      dim: [
        { label: "column_index", size: cols, stride: rows * cols },
        { label: "row_index", size: rows, stride: rows },
      ],
      data_offset: 0,
    },
    data,
  };
}

function GridMapStory({
  rows = 50,
  cols = 50,
  outerStartIndex = 0,
  innerStartIndex = 0,
}: {
  rows?: number;
  cols?: number;
  outerStartIndex?: number;
  innerStartIndex?: number;
} = {}): JSX.Element {
  const topics: Topic[] = [
    { name: "/grid_map", schemaName: "grid_map_msgs/msg/GridMap" },
    { name: "/tf", schemaName: "geometry_msgs/TransformStamped" },
  ];

  const resolution = 0.1;

  // rows run along x (forward), cols along y (left)
  const elevationLayer = createGridMapLayer(
    rows,
    cols,
    (i, j) => {
      const x = (i - rows / 2) * resolution;
      const y = (j - cols / 2) * resolution;
      return 0.5 * Math.sin(x * 0.5) * Math.cos(y * 0.5);
    },
    outerStartIndex,
    innerStartIndex,
  );

  const intensityLayer = createGridMapLayer(
    rows,
    cols,
    (i, j) => {
      const x = (i - rows / 2) / rows;
      const y = (j - cols / 2) / cols;
      return (x * x + y * y) * 2;
    },
    outerStartIndex,
    innerStartIndex,
  );

  const gridMap: MessageEvent<GridMap> = {
    topic: "/grid_map",
    schemaName: "grid_map_msgs/msg/GridMap",
    receiveTime: { sec: 10, nsec: 0 },
    message: {
      info: {
        header: { frame_id: "map", stamp: { sec: 0, nsec: 0 } },
        resolution,
        length_x: rows * resolution,
        length_y: cols * resolution,
        pose: {
          position: { x: 0, y: 0, z: 0 },
          orientation: QUAT_IDENTITY,
        },
      },
      layers: ["elevation", "intensity"],
      basic_layers: ["elevation"],
      data: [elevationLayer, intensityLayer],
      outer_start_index: outerStartIndex,
      inner_start_index: innerStartIndex,
    },
    sizeInBytes: 0,
  };

  const tf: MessageEvent<TransformStamped> = {
    topic: "/tf",
    schemaName: "geometry_msgs/TransformStamped",
    receiveTime: { sec: 10, nsec: 0 },
    message: {
      header: { frame_id: "map", stamp: { sec: 0, nsec: 0 } },
      child_frame_id: "base_link",
      transform: {
        translation: { x: 0, y: 0, z: 0 },
        rotation: QUAT_IDENTITY,
      },
    },
    sizeInBytes: 0,
  };

  const fixture = useDelayedFixture({
    topics,
    frame: {
      "/grid_map": [gridMap],
      "/tf": [tf],
    },
  });

  return (
    <PanelSetup fixture={fixture}>
      <ThreeDeePanel />
    </PanelSetup>
  );
}

export const GridMapRender: StoryObj = {
  render: () => <GridMapStory />,
};

/** Non-square map with a scrolled circular buffer (non-zero start indices). */
export const GridMapScrolledNonSquare: StoryObj = {
  render: () => <GridMapStory rows={40} cols={60} outerStartIndex={7} innerStartIndex={13} />,
};
