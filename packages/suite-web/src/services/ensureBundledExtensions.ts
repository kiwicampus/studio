// SPDX-FileCopyrightText: Copyright (C) 2023-2026 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

import Log from "@lichtblick/log";
import { IExtensionLoader } from "@lichtblick/suite-base/services/extension/IExtensionLoader";

const log = Log.getLogger(__filename);

// The app doesn't render anything until this module's install pass settles (see WebRoot.tsx), so
// a hung fetch (dead proxy, flaky network) must not be able to block app boot indefinitely.
const FETCH_TIMEOUT_MS = 5_000;

// Ids that have been successfully seeded at least once, so repeat page loads (the overwhelming
// majority) can skip the IndexedDB/network round-trip entirely via areBundledExtensionsSeeded().
const SEEDED_EXTENSIONS_STORAGE_KEY = "lichtblick.bundledExtensionsSeeded";

type BundledExtension = {
  /** id the extension loader assigns on install (`${publisher}.${name}`), used to skip reinstalling */
  id: string;
  /** path to the .foxe file, served from the web app's own static assets */
  url: string;
};

/**
 * Extensions installed automatically into the "local" namespace on first web app load.
 *
 * The Marketplace install flow is desktop-only (see ExtensionDetails.tsx), so extensions that
 * ship with this fork's web build need to be seeded directly into extension storage instead.
 */
const BUNDLED_EXTENSIONS: BundledExtension[] = [
  { id: "Cloudini.foxglove-cloudini-converter", url: "extensions/cloudini.foxe" },
];

function readSeededIds(): Set<string> {
  try {
    const raw = localStorage.getItem(SEEDED_EXTENSIONS_STORAGE_KEY);
    return raw ? new Set(JSON.parse(raw) as string[]) : new Set();
  } catch {
    return new Set();
  }
}

/**
 * True once every bundled extension has been seeded at least once. WebRoot uses this as a
 * synchronous fast path so the common case (already seeded) never blocks the first render on an
 * IndexedDB round-trip.
 */
export function areBundledExtensionsSeeded(): boolean {
  const seededIds = readSeededIds();
  return BUNDLED_EXTENSIONS.every((extension) => seededIds.has(extension.id));
}

/**
 * Installs any bundled extension not already present in `localLoader` (idempotent — safe to call
 * on every app boot). Failures are logged and swallowed so a missing/broken bundled extension
 * never blocks app startup; unlike a success, a failure isn't recorded as seeded, so it's retried
 * on the next load.
 */
export async function ensureBundledExtensionsInstalled(
  localLoader: IExtensionLoader,
): Promise<void> {
  const seededIds = readSeededIds();

  await Promise.all(
    BUNDLED_EXTENSIONS.filter((extension) => !seededIds.has(extension.id)).map(
      async (extension) => {
        try {
          if (await localLoader.getExtension(extension.id)) {
            seededIds.add(extension.id);
            return;
          }

          // Fetched from this web app's own static assets (packages/suite-web/public), not the
          // configured API host, so this deliberately bypasses HttpService — whose baseURL is
          // APP_CONFIG.apiUrl — in favor of a plain same-origin fetch.
          const response = await fetch(extension.url, {
            signal: AbortSignal.timeout(FETCH_TIMEOUT_MS),
          });
          if (!response.ok) {
            throw new Error(`${response.status} ${response.statusText}`);
          }
          const foxeFileData = new Uint8Array(await response.arrayBuffer());
          await localLoader.installExtension({ foxeFileData });
          log.info(`Installed bundled extension ${extension.id}`);
          seededIds.add(extension.id);
        } catch (error) {
          log.error(`Failed to install bundled extension ${extension.id}`, error);
        }
      },
    ),
  );

  try {
    localStorage.setItem(SEEDED_EXTENSIONS_STORAGE_KEY, JSON.stringify([...seededIds]) ?? "");
  } catch (error) {
    log.error("Failed to persist bundled extension seed state", error);
  }
}
