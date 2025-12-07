// SPDX-FileCopyrightText: Copyright (C) 2023-2025 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)<lichtblick@bmwgroup.com>
// SPDX-License-Identifier: MPL-2.0

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/

/* eslint-disable filenames/match-exported */

interface Topic {
  [name: string]: string;
}

interface TypeIndex {
  [type: string]: string | undefined;
}

interface SubscribePayload {
  topicName: string;
  maxUpdateRate: number;
}

interface UnsubscribePayload {
  topicName: string;
}

interface MessagePayload {
  [key: string]: unknown;
}

type MessageCallback = (message: MessagePayload) => void;
type EventCallback = () => void;

// Replaces any key named "nsec" with "nanosec" in the object
function renameNsecToNanosec(obj: unknown): unknown {
  if (Array.isArray(obj)) {
    return obj.map((item) => renameNsecToNanosec(item));
  } else if (obj != undefined && typeof obj === "object") {
    const newObj: Record<string, unknown> = {};
    for (const key in obj) {
      if (Object.prototype.hasOwnProperty.call(obj, key)) {
        const newKey = key === "nsec" ? "nanosec" : key;
        newObj[newKey] = renameNsecToNanosec((obj as Record<string, unknown>)[key]);
      }
    }
    return newObj;
  }
  return obj;
}

export class PubTopic {
  public rosClient: RosboardClient;
  public name: string;
  public messageType: string;
  public queueSize: number;

  public constructor(
    rosClient: RosboardClient,
    name: string,
    messageType: string,
    queueSize: number,
  ) {
    this.rosClient = rosClient;
    this.name = name;
    this.messageType = messageType;
    this.queueSize = queueSize;
  }

  public unadvertise(): void {
    // Send message to destroy publisher in server. Message is expected to be: ["n", {topicName: xxxx}]
    const message = JSON.stringify(["n", { topicName: this.name }]);
    if (message != undefined) {
      this.rosClient.send(message);
    } else {
      console.error("Message is undefined. Cannot send.");
    }
  }

  // Just for compatibility with the roslib version
  // In rosboard we don't need to advertise the topic, it is done automatically when
  // we send a message for the first time in the rosboard server
  public advertise(): void {}

  public publish(msg: MessagePayload): void {
    // Rosboard expects headers to have nsec named nanosec, but foxglove uses nsec
    const renamedMsg = renameNsecToNanosec(msg) as MessagePayload;
    // rosboard expects a message like this: ["m", {message dictionary}]
    // and message dictionary is in the form of {_topic_name: topic, _topic_type: type, ...payload}
    // eslint-disable-next-line @typescript-eslint/naming-convention
    const payload: Record<string, unknown> = {
      // eslint-disable-next-line @typescript-eslint/naming-convention
      _topic_name: this.name,
      // eslint-disable-next-line @typescript-eslint/naming-convention
      _topic_type: this.messageType,
      ...renamedMsg,
    };

    const jsonString = JSON.stringify(["m", payload]);

    if (jsonString != undefined) {
      this.rosClient.send(jsonString);
    } else {
      console.error("Message is undefined. Cannot send.");
    }
  }
}

class RosboardClient {
  public ws?: WebSocket;
  public hostname: string = "";
  public version: string = "";
  public closed: boolean = false;
  public url: string;
  public auto_reconnect: boolean = true;
  private _availableTopics: Topic = {};
  private _topicsFull: TypeIndex = {};
  private _topicsFullRequested: boolean = false;
  public sequenceNumber: number | undefined = undefined;
  public connectionCallbacks: EventCallback[] = [];
  public errorCallback?: (error: Error) => void;
  public closeCallback?: () => void;
  public subscribedTopics: string[] = [];
  public topicCallbacks: { [topicName: string]: MessageCallback } = {};

  public constructor({ url }: { url: string }) {
    this.url = url;
    this.openConnection();
  }

  public openConnection = (): void => {
    if (this.ws != undefined) {
      throw new Error(`Attempted to open a second WebSocket Connection`);
    }

    const ws = new WebSocket(this.url);

    ws.addEventListener("open", () => {
      this.ws = ws;
      this.connectionCallbacks.forEach((callback) => {
        callback();
      });
    });

    ws.addEventListener("error", (event) => {
      console.error("WebSocket error:", event);
      const error =
        event instanceof ErrorEvent && event.error instanceof Error
          ? event.error
          : new Error("WebSocket error");
      if (this.errorCallback) {
        this.errorCallback(error);
      }
    });

    ws.addEventListener("close", () => {
      this.ws = undefined;
      this.closed = true;
      if (this.closeCallback) {
        this.closeCallback();
      }
    });

    ws.addEventListener("message", async (event) => {
      try {
        let data: [string, unknown];
        if (typeof event.data === "string") {
          data = JSON.parse(event.data) as [string, unknown];
        } else if (event.data instanceof Blob) {
          const result = await new Promise<string>((resolve, reject) => {
            const reader = new FileReader();

            reader.onload = () => {
              if (reader.result != null) {
                resolve(reader.result as string);
              } else {
                reject(new Error("Reader result is empty"));
              }
            };

            reader.onerror = () => {
              reject(reader.error ?? new Error("FileReader error"));
            };

            reader.readAsText(event.data as Blob);
          });

          data = JSON.parse(result) as [string, unknown];
        } else {
          data = ["z", "invalid"];
          //console.log("Invalid input");
          //console.log(typeof(data));
        }
        const [type, payload] = data;

        if (type === "y" && typeof payload === "object" && payload != undefined) {
          const yPayload = payload as Record<string, unknown>;
          const hostnameValue = yPayload.hostname;
          const versionValue = yPayload.version;
          this.hostname =
            typeof hostnameValue === "string"
              ? hostnameValue
              : hostnameValue != undefined
                ? String(hostnameValue)
                : "";
          this.version =
            typeof versionValue === "string"
              ? versionValue
              : versionValue != undefined
                ? String(versionValue)
                : "";
          if ("auto_reconnect" in yPayload) {
            this.auto_reconnect = Boolean(yPayload.auto_reconnect);
          }
        } else if (type === "t" && typeof payload === "object" && payload != undefined) {
          // Update availableTopics directly with the new payload
          this._availableTopics = payload as Topic;
          //console.log('Updated Available Topics:', this._availableTopics);
        } else if (type === "f" && typeof payload === "object" && payload != undefined) {
          // Update availableTopics directly with the new payload
          const typedefs: TypeIndex = {};
          const fPayload = payload as Record<string, { type: string; typedef: string }>;
          Object.keys(fPayload).forEach((k) => {
            const item = fPayload[k];
            if (item?.type != undefined) {
              typedefs[item.type] = item.typedef;
            }
          });
          this._topicsFull = typedefs;
          this._topicsFullRequested = false;
        } else if (
          type === "m" &&
          typeof payload === "object" &&
          payload != undefined &&
          // eslint-disable-next-line no-underscore-dangle
          "_topic_name" in payload &&
          // eslint-disable-next-line no-underscore-dangle
          typeof (payload as { _topic_name: unknown })._topic_name === "string" &&
          // eslint-disable-next-line no-underscore-dangle
          this.subscribedTopics.includes(String((payload as { _topic_name: string })._topic_name))
        ) {
          // Message received for a subscribed topic
          // eslint-disable-next-line no-underscore-dangle
          const topicName = String((payload as { _topic_name: string })._topic_name);
          if (this.topicCallbacks[topicName]) {
            // Execute the callback function for the topic
            const callback = this.topicCallbacks[topicName];
            if (typeof callback === "function") {
              callback(payload as MessagePayload);
            }
          }
        } else if (
          type === "p" &&
          typeof payload === "object" &&
          payload != undefined &&
          typeof (payload as { s?: unknown }).s === "number"
        ) {
          // Respond with a message of type 'q' containing the current timestamp and matching sequence number
          const sequenceNumber = (payload as { s: number }).s;
          const timestamp = Date.now();
          const response = JSON.stringify(["q", { s: sequenceNumber, t: timestamp }]);
          if (this.ws != undefined && response != undefined) {
            this.ws.send(response);
            //console.log('Sent response:', response);
          }
        }
      } catch (error: unknown) {
        console.error("Error parsing message:", error);
      }
    });
  };

  public on(
    event: "connection" | "error" | "close",
    callback: EventCallback | ((error: Error) => void) | (() => void),
  ): void {
    if (event === "connection") {
      this.connectionCallbacks.push(callback as EventCallback);
    } else if (event === "error") {
      this.errorCallback = callback as (error: Error) => void;
    } else {
      // event === "close"
      this.closeCallback = callback as () => void;
    }
  }

  public availableTopics(): Topic {
    return this._availableTopics;
  }

  public topicsFull(): TypeIndex {
    return this._topicsFull;
  }

  public requestTopicsFull(): void {
    if (this._topicsFullRequested) {
      return;
    }
    const message = JSON.stringify(["f"]);
    if (message != undefined) {
      this.send(message);
      this._topicsFullRequested = true;
    } else {
      console.error("Message is undefined. Cannot send.");
    }
  }

  public subscribe(topicName: string, maxUpdateRate: number): void {
    const payload: SubscribePayload = {
      topicName,
      maxUpdateRate,
    };
    const message = JSON.stringify(["s", payload]);
    //console.log(message);
    //console.log ("Subscribing to ", topicName);
    if (message != undefined) {
      this.send(message);
    } else {
      console.error("Message is undefined. Cannot send.");
    }
    this.subscribedTopics.push(topicName);
  }

  public unsubscribe(topicName: string): void {
    const payload: UnsubscribePayload = {
      topicName,
    };
    const message = JSON.stringify(["u", payload]);
    //console.log ("Un-Subscribing to ", topicName);
    if (message != undefined) {
      this.send(message);
    } else {
      console.error("Message is undefined. Cannot send.");
    }
    const index = this.subscribedTopics.indexOf(topicName);
    if (index !== -1) {
      this.subscribedTopics.splice(index, 1);
    }
  }

  public addTopicCallback(topicName: string, callback: MessageCallback): void {
    this.topicCallbacks[topicName] = callback;
    // Subscribe to the topic when adding the callback
    this.subscribe(topicName, 24);
  }

  public send(message: string): void {
    if (this.ws != undefined) {
      this.ws.send(message);
      //console.log('Sent message:', message);
    } else {
      console.error("WebSocket connection is not established");
    }
  }

  public close(): void {
    if (this.ws != undefined) {
      this.ws.close();
      this.ws = undefined;
      this.closed = true;
      //console.log('WebSocket connection closed');
    } else {
      console.warn("WebSocket connection is already closed");
    }
  }
}

export { RosboardClient };
export default RosboardClient;
