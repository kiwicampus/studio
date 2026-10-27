// SPDX-FileCopyrightText: Copyright (C) 2023-2026 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

import { buildSettingsTree, Config } from "@lichtblick/suite-base/panels/Map/config";

describe("buildSettingsTree overlay", () => {
  const config: Config = {
    customTileUrl: "",
    disabledTopics: [],
    followTopic: "",
    layer: "satellite",
    topicColors: {},
  };

  it("only shows the overlay options once an overlay url is set", () => {
    const empty = buildSettingsTree(config, []).general!.fields!;
    expect(empty.overlayTileUrl).toBeDefined();
    expect(empty.overlayOpacity).toBeUndefined();
    expect(empty.maxNativeZoom).toBeUndefined();

    const fields = buildSettingsTree(
      { ...config, overlayTileUrl: "https://example.com/{z}/{x}/{y}.png" },
      [],
    ).general!.fields!;
    expect(fields.overlayOpacity).toMatchObject({ value: 1, min: 0, max: 1 });
    expect(fields.maxNativeZoom).toBeDefined();
  });

  it("reports invalid placeholders in the overlay url", () => {
    const fields = buildSettingsTree(
      { ...config, overlayTileUrl: "https://example.com/{s}/{z}/{x}/{y}.png" },
      [],
    ).general!.fields!;
    expect(fields.overlayTileUrl?.error).toBe("Invalid placeholder {s}");
  });
});
