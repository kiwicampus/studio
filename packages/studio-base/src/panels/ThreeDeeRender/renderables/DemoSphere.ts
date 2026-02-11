// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

import { t } from "i18next";
import * as _ from "lodash-es";

import Logger from "@foxglove/log";
import { SettingsTreeAction, SettingsTreeFields } from "@foxglove/studio";

import { RenderableSphere } from "./markers/RenderableSphere";
import type { IRenderer } from "../IRenderer";
import { BaseUserData, Renderable } from "../Renderable";
import { SceneExtension } from "../SceneExtension";
import { SettingsTreeEntry } from "../SettingsManager";
import { stringToRgba } from "../color";
import { vec3TupleApproxEquals } from "../math";
import { Marker, MarkerAction, MarkerType, TIME_ZERO } from "../ros";
import { CustomLayerSettings, PRECISION_DEGREES, PRECISION_DISTANCE } from "../settings";
import { makePose, xyzrpyToPose } from "../transforms";

const log = Logger.getLogger(__filename);

export type LayerSettingsDemoSphere = CustomLayerSettings & {
  layerId: "foxglove.DemoSphere";
  frameId: string | undefined;
  radius: number;
  color: string;
  position: [number, number, number];
  rotation: [number, number, number];
};

const LAYER_ID = "foxglove.DemoSphere";
const DEFAULT_RADIUS = 0.5;
const DEFAULT_COLOR = "#9c27b0";

const DEFAULT_SETTINGS: LayerSettingsDemoSphere = {
  visible: true,
  frameLocked: true,
  label: "Demo Sphere",
  instanceId: "invalid",
  layerId: LAYER_ID,
  frameId: undefined,
  radius: DEFAULT_RADIUS,
  color: DEFAULT_COLOR,
  position: [0, 0, 1],
  rotation: [0, 0, 0],
};

export type DemoSphereUserData = BaseUserData & {
  settings: LayerSettingsDemoSphere;
  sphere: RenderableSphere;
};

export class DemoSphereRenderable extends Renderable<DemoSphereUserData> {
  public override dispose(): void {
    this.userData.sphere.dispose();
    super.dispose();
  }
}

export class DemoSphere extends SceneExtension<DemoSphereRenderable> {
  public static extensionId = "foxglove.DemoSphere";

  public constructor(renderer: IRenderer, name: string = DemoSphere.extensionId) {
    super(name, renderer);

    renderer.addCustomLayerAction({
      layerId: LAYER_ID,
      label: t("threeDee:addDemoSphere"),
      icon: "Circle",
      handler: this.#handleAddDemoSphere,
    });

    renderer.on("transformTreeUpdated", this.#handleTransformTreeUpdated);

    for (const [instanceId, entry] of Object.entries(renderer.config.layers)) {
      if (entry?.layerId === LAYER_ID) {
        this.#updateSphere(instanceId, entry as Partial<LayerSettingsDemoSphere>);
      }
    }
  }

  public override dispose(): void {
    this.renderer.off("transformTreeUpdated", this.#handleTransformTreeUpdated);
    super.dispose();
  }

  public override removeAllRenderables(): void {
    // no-op
  }

  public override settingsNodes(): SettingsTreeEntry[] {
    const handler = this.handleSettingsAction;
    const entries: SettingsTreeEntry[] = [];
    for (const [instanceId, layerConfig] of Object.entries(this.renderer.config.layers)) {
      if (layerConfig?.layerId !== LAYER_ID) {
        continue;
      }

      const config = layerConfig as Partial<LayerSettingsDemoSphere>;
      const frameIdOptions = [
        { label: "<Display frame>", value: undefined },
        ...this.renderer.coordinateFrameList,
      ];

      const fields: SettingsTreeFields = {
        frameId: {
          label: t("threeDee:frame"),
          input: "select",
          options: frameIdOptions,
          value: config.frameId,
        },
        radius: {
          label: t("threeDee:radius"),
          input: "number",
          min: 0.01,
          step: 0.1,
          precision: PRECISION_DISTANCE,
          value: config.radius ?? DEFAULT_RADIUS,
          placeholder: String(DEFAULT_RADIUS),
        },
        color: {
          label: t("threeDee:color"),
          input: "rgba",
          value: config.color ?? DEFAULT_COLOR,
        },
        position: {
          label: t("threeDee:position"),
          input: "vec3",
          labels: ["X", "Y", "Z"],
          precision: PRECISION_DISTANCE,
          value: config.position ?? [0, 0, 1],
        },
        rotation: {
          label: t("threeDee:rotation"),
          input: "vec3",
          labels: ["R", "P", "Y"],
          precision: PRECISION_DEGREES,
          value: config.rotation ?? [0, 0, 0],
        },
      };

      entries.push({
        path: ["layers", instanceId],
        node: {
          label: config.label ?? t("threeDee:demoSphere"),
          icon: "Circle",
          fields,
          visible: config.visible ?? DEFAULT_SETTINGS.visible,
          actions: [{ type: "action", id: "delete", label: t("threeDee:delete") }],
          order: layerConfig.order,
          handler,
        },
      });

      if (!this.renderables.has(instanceId)) {
        this.#updateSphere(instanceId, config);
      }
    }
    return entries;
  }

  public override startFrame(
    currentTime: bigint,
    renderFrameId: string,
    fixedFrameId: string,
  ): void {
    for (const renderable of this.renderables.values()) {
      renderable.userData.frameId =
        renderable.userData.settings.frameId ?? renderFrameId;
    }
    super.startFrame(currentTime, renderFrameId, fixedFrameId);
  }

  public override handleSettingsAction = (action: SettingsTreeAction): void => {
    const path = action.payload.path;

    if (action.action === "perform-node-action") {
      if (path.length === 2 && action.payload.id === "delete") {
        const instanceId = path[1]!;
        this.renderer.updateConfig((draft) => {
          delete draft.layers[instanceId];
        });
        this.#updateSphere(instanceId, undefined);
        this.updateSettingsTree();
        this.renderer.updateCustomLayersCount();
      }
      return;
    }

    if (path.length !== 3) {
      return;
    }

    this.saveSetting(path, action.payload.value);

    const instanceId = path[1]!;
    const settings = this.renderer.config.layers[instanceId] as
      | Partial<LayerSettingsDemoSphere>
      | undefined;
    this.#updateSphere(instanceId, settings);
  };

  #handleAddDemoSphere = (instanceId: string): void => {
    log.info(`Creating ${LAYER_ID} layer ${instanceId}`);

    const config: LayerSettingsDemoSphere = { ...DEFAULT_SETTINGS, instanceId };

    this.renderer.updateConfig((draft) => {
      const maxOrderLayer = _.maxBy(Object.values(draft.layers), (layer) => layer?.order);
      const order = 1 + (maxOrderLayer?.order ?? 0);
      draft.layers[instanceId] = { ...config, order };
    });

    this.#updateSphere(instanceId, config);
    this.updateSettingsTree();
  };

  #handleTransformTreeUpdated = (): void => {
    this.updateSettingsTree();
  };

  #updateSphere(
    instanceId: string,
    settings: Partial<LayerSettingsDemoSphere> | undefined,
  ): void {
    let renderable = this.renderables.get(instanceId);

    if (settings == undefined) {
      if (renderable != undefined) {
        renderable.userData.sphere.dispose();
        this.remove(renderable);
        this.renderables.delete(instanceId);
      }
      return;
    }

    const newSettings = { ...DEFAULT_SETTINGS, ...settings };
    if (!renderable) {
      renderable = this.#createRenderable(instanceId, newSettings);
      renderable.userData.pose = xyzrpyToPose(newSettings.position, newSettings.rotation);
    }

    const prevSettings = renderable.userData.settings;
    const markerChanged =
      newSettings.radius !== prevSettings.radius ||
      newSettings.color !== prevSettings.color ||
      newSettings.frameId !== prevSettings.frameId;

    renderable.userData.settings = newSettings;

    if (markerChanged) {
      const marker = createSphereMarker(newSettings);
      renderable.userData.sphere.update(marker, undefined);
    }

    if (
      !vec3TupleApproxEquals(newSettings.position, prevSettings.position) ||
      !vec3TupleApproxEquals(newSettings.rotation, prevSettings.rotation)
    ) {
      renderable.userData.pose = xyzrpyToPose(newSettings.position, newSettings.rotation);
    }
  }

  #createRenderable(instanceId: string, settings: LayerSettingsDemoSphere): DemoSphereRenderable {
    const marker = createSphereMarker(settings);
    const sphere = new RenderableSphere(
      `demo:${instanceId}`,
      marker,
      undefined,
      this.renderer,
    );
    const renderable = new DemoSphereRenderable(instanceId, this.renderer, {
      receiveTime: 0n,
      messageTime: 0n,
      frameId: "",
      pose: makePose(),
      settingsPath: ["layers", instanceId],
      settings,
      sphere,
    });
    renderable.add(sphere);

    this.add(renderable);
    this.renderables.set(instanceId, renderable);
    return renderable;
  }
}

/** Creates a visualization_msgs/Marker compatible sphere marker.
 * Uses MarkerType.SPHERE (2) - the standard ROS visualization_msgs/Marker sphere type.
 */
function createSphereMarker(settings: LayerSettingsDemoSphere): Marker {
  const color = { r: 1, g: 1, b: 1, a: 0.8 };
  stringToRgba(color, settings.color);

  const diameter = settings.radius * 2;
  return {
    header: {
      frame_id: "",
      stamp: TIME_ZERO,
    },
    ns: "demo",
    id: 0,
    type: MarkerType.SPHERE,
    action: MarkerAction.ADD,
    pose: makePose(),
    scale: { x: diameter, y: diameter, z: diameter },
    color,
    lifetime: TIME_ZERO,
    frame_locked: true,
    points: [],
    colors: [],
    text: "",
    mesh_resource: "",
    mesh_use_embedded_materials: false,
  };
}
