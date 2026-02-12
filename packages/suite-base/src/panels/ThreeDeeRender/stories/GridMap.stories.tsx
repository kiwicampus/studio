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

function createGridMapLayer(
  rows: number,
  cols: number,
  generator: (i: number, j: number) => number,
): { layout: { dim: { label: string; size: number; stride: number }[]; data_offset: number }; data: Float32Array } {
  const data = new Float32Array(rows * cols);
  for (let i = 0; i < rows; i++) {
    for (let j = 0; j < cols; j++) {
      data[i * cols + j] = generator(i, j);
    }
  }
  return {
    layout: {
      dim: [
        { label: "row_index", size: rows, stride: rows * cols },
        { label: "column_index", size: cols, stride: cols },
      ],
      data_offset: 0,
    },
    data,
  };
}

function GridMapStory(): JSX.Element {
  const topics: Topic[] = [
    { name: "/grid_map", schemaName: "grid_map_msgs/msg/GridMap" },
    { name: "/tf", schemaName: "geometry_msgs/TransformStamped" },
  ];

  const rows = 50;
  const cols = 50;
  const resolution = 0.1;

  const elevationLayer = createGridMapLayer(rows, cols, (i, j) => {
    const x = (j - cols / 2) * resolution;
    const y = (i - rows / 2) * resolution;
    return 0.5 * Math.sin(x * 0.5) * Math.cos(y * 0.5);
  });

  const intensityLayer = createGridMapLayer(rows, cols, (i, j) => {
    const x = (j - cols / 2) / cols;
    const y = (i - rows / 2) / rows;
    return (x * x + y * y) * 2;
  });

  const gridMap: MessageEvent<GridMap> = {
    topic: "/grid_map",
    schemaName: "grid_map_msgs/msg/GridMap",
    receiveTime: { sec: 10, nsec: 0 },
    message: {
      info: {
        header: { frame_id: "map", stamp: { sec: 0, nsec: 0 } },
        resolution,
        length_x: cols * resolution,
        length_y: rows * resolution,
        pose: {
          position: { x: 0, y: 0, z: 0 },
          orientation: QUAT_IDENTITY,
        },
      },
      layers: ["elevation", "intensity"],
      basic_layers: ["elevation"],
      data: [elevationLayer, intensityLayer],
      outer_start_index: 0,
      inner_start_index: 0,
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
