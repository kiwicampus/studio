// SPDX-FileCopyrightText: Copyright (C) 2023-2026 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import { useEffect, useMemo, useState } from "react";

import Log from "@lichtblick/log";
import {
  AppBarProps,
  AppSetting,
  IExtensionLoader,
  FoxgloveWebSocketDataSourceFactory,
  IDataSourceFactory,
  IdbExtensionLoader,
  McapLocalDataSourceFactory,
  RemoteDataSourceFactory,
  RemoteExtensionLoader,
  Ros1LocalBagDataSourceFactory,
  Ros2LocalBagDataSourceFactory,
  RosbridgeDataSourceFactory,
  RosboardDataSourceFactory,
  SampleNuscenesDataSourceFactory,
  SharedRoot,
  UlogLocalDataSourceFactory,
} from "@lichtblick/suite-base";
import { APP_CONFIG } from "@lichtblick/suite-base/constants/config";

import LocalStorageAppConfiguration from "./services/LocalStorageAppConfiguration";
import {
  areBundledExtensionsSeeded,
  ensureBundledExtensionsInstalled,
} from "./services/ensureBundledExtensions";

const log = Log.getLogger(__filename);

const isDevelopment = process.env.NODE_ENV === "development";

export function WebRoot(props: {
  extraProviders: React.JSX.Element[] | undefined;
  dataSources: IDataSourceFactory[] | undefined;
  AppBarComponent?: (props: AppBarProps) => React.JSX.Element;
  children: React.JSX.Element;
}): React.JSX.Element | ReactNull {
  const appConfiguration = useMemo(
    () =>
      new LocalStorageAppConfiguration({
        defaults: {
          [AppSetting.SHOW_DEBUG_PANELS]: isDevelopment,
        },
      }),
    [],
  );

  const defaultExtensionLoaders: IExtensionLoader[] = [
    new IdbExtensionLoader("org"),
    new IdbExtensionLoader("local"),
  ];
  const url = new URL(window.location.href);
  const workspace = url.searchParams.get("workspace");

  if (workspace && APP_CONFIG.apiUrl) {
    defaultExtensionLoaders.push(new RemoteExtensionLoader("org", workspace));
  }
  const [extensionLoaders] = useState(() => defaultExtensionLoaders);

  // Seed bundled extensions (e.g. the Cloudini compressed point cloud converter) into the local
  // extension store before the app mounts, since installing from the Marketplace is desktop-only.
  // Skipped on the (overwhelmingly common) already-seeded path so repeat loads render immediately.
  const [bundledExtensionsReady, setBundledExtensionsReady] = useState(areBundledExtensionsSeeded);
  useEffect(() => {
    if (bundledExtensionsReady) {
      return;
    }

    // "local" is always present in defaultExtensionLoaders above.
    const localLoader = extensionLoaders.find((loader) => loader.namespace === "local")!;
    ensureBundledExtensionsInstalled(localLoader)
      .catch((error: unknown) => {
        log.error("Failed to install bundled extensions", error);
      })
      .finally(() => {
        setBundledExtensionsReady(true);
      });
  }, [bundledExtensionsReady, extensionLoaders]);

  const dataSources = useMemo(() => {
    const sources = [
      new Ros1LocalBagDataSourceFactory(),
      new Ros2LocalBagDataSourceFactory(),
      new RosboardDataSourceFactory(),
      new FoxgloveWebSocketDataSourceFactory(),
      new RosbridgeDataSourceFactory(),
      new UlogLocalDataSourceFactory(),
      new SampleNuscenesDataSourceFactory(),
      new McapLocalDataSourceFactory(),
      new RemoteDataSourceFactory(),
    ];

    return props.dataSources ?? sources;
  }, [props.dataSources]);

  if (!bundledExtensionsReady) {
    return ReactNull;
  }

  return (
    <SharedRoot
      enableLaunchPreferenceScreen
      deepLinks={[window.location.href]}
      dataSources={dataSources}
      appConfiguration={appConfiguration}
      extensionLoaders={extensionLoaders}
      enableGlobalCss
      extraProviders={props.extraProviders}
      AppBarComponent={props.AppBarComponent}
    >
      {props.children}
    </SharedRoot>
  );
}
