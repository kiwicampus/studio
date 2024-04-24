interface Topic {
    [name: string]: string;
}

interface SubscribePayload {
    topicName: string;
    maxUpdateRate: number;
}

interface UnsubscribePayload {
    topicName: string;
}

interface MessagePayload {
    [key: string]: any;
}

type MessageCallback = (message: MessagePayload) => void;
type EventCallback = () => void;

export default class RosboardClient {
    ws?: WebSocket;
    hostname: string = '';
    version: string = '';
    closed: boolean = false;
    url: string;
    private _availableTopics: Topic = {};
    sequenceNumber: number | null = null;
    connectionCallbacks: EventCallback[] = [];
    errorCallback?: (error: Error) => void;
    closeCallback?: () => void;
    subscribedTopics: string[] = [];
    topicCallbacks: { [topicName: string]: MessageCallback } = {};

    public constructor({
        url,
    }: {
        url: string;
    }) {
        this.url = url;
        this.openConnection();
    }

    openConnection = (): void => {
        if (this.ws != undefined) {
            throw new Error(`Attempted to open a second WebSocket Connection`);
        }

	console.log("connection stablished");
        const ws = new WebSocket(this.url);

        ws.addEventListener('open', () => {
            this.ws = ws;
            this.connectionCallbacks.forEach(callback => callback());
        });

        ws.addEventListener('error', (event) => {
	    console.error('WebSocket error:', event);
	    const error = event instanceof ErrorEvent ? event.error : new Error('WebSocket error');
	    if (this.errorCallback) {
		this.errorCallback(error);
	    }
        });

        ws.addEventListener('close', () => {
            console.log('WebSocket connection closed');
            this.ws = undefined;
            this.closed = true;
            if (this.closeCallback) {
                this.closeCallback();
            }
        });

        ws.addEventListener('message', async (event) => {
	    
	    console.log ("MESSAGE");
	    try {
		let data: [string, any];
		if (typeof event.data === 'string') {
		    data = JSON.parse(event.data as string);
		} else if (event.data instanceof Blob) {
		    const result = await new Promise<string>((resolve, reject) => {
                        const reader = new FileReader();

                        reader.onload = () => {
                            if (reader.result) {
                                resolve(reader.result as string);
                            } else {
                                reject(new Error("Reader result is empty"));
                            }
                        };

                        reader.onerror = () => {
                            reject(reader.error);
                        };

                        reader.readAsText(event.data as Blob);
                    });

                    data = JSON.parse(result) as [string, any];
		} else {
		    data = ["z", "invalid"];
		    console.log("Invalid input");
		    console.log(typeof(data));
		}
		const [type, payload] = data;

                if (type === 'y' && typeof payload === 'object') {
                    this.hostname = payload.hostname;
                    this.version = payload.version;

                    console.log('Hostname:', this.hostname);
                    console.log('Version:', this.version);
                } else if (type === 't' && typeof payload === 'object') {
                    // Update availableTopics directly with the new payload
                    this._availableTopics = payload;
                    // console.log('Updated Available Topics:', this._availableTopics);
                } else if (type === 'm' && typeof payload === 'object' && payload._topic_name && this.subscribedTopics.includes(payload._topic_name)) {
                    // Message received for a subscribed topic
                    const topicName = payload._topic_name;
                    if (this.topicCallbacks[topicName]) {
                        // Execute the callback function for the topic
                        const callback = this.topicCallbacks[topicName];
			if (typeof callback == 'function') {
                            callback(payload);
			}
                    }
                } else if (type === 'p' && typeof payload === 'object' && typeof payload.s === 'number') {
                    // Respond with a message of type 'q' containing the current timestamp and matching sequence number
                    const sequenceNumber = payload.s;
                    const timestamp = Date.now();
                    const response = JSON.stringify(['q', { s: sequenceNumber, t: timestamp }]);
                    if (this.ws && response) {
			const buffer = Buffer.from(response);
                        this.ws.send(response);
                        // console.log('Sent response:', response);
                    }
                } 
            } catch (error) {
                console.error('Error parsing message:', error);
            }
        });
    }

    on(event: 'connection' | 'error' | 'close', callback: EventCallback | ((error: Error) => void) | (() => void)) {
        if (event === 'connection') {
            this.connectionCallbacks.push(callback as EventCallback);
        } else if (event === 'error') {
            this.errorCallback = callback as (error: Error) => void;
        } else if (event === 'close') {
            this.closeCallback = callback as () => void;
        }
    }

    get availableTopics(): Topic {
        return this._availableTopics;
    }

    getAvailableTopics(): Promise<Topic> {
	    return new Promise((resolve, reject) => {
		const requestMessage = JSON.stringify(['get_available_topics']);
		if (requestMessage !== undefined) {
		    this.send(requestMessage);
		} else {
		    reject(new Error("Failed to stringify request message."));
		}

		const onResponse = (data: string | any[]) => {
		    try {
			let jsonData: any;
			if (typeof data === 'string') {
			    jsonData = JSON.parse(data);
			} else if (Array.isArray(data)) {
			    jsonData = data;
			} else {
			    reject(new Error('Unexpected data type received'));
			    return;
			}

			const [type, payload] = jsonData;
			if (type === 't' && typeof payload === 'object') {
			    resolve(payload);
			} else {
			    reject(new Error('Invalid response from server'));
			}
		    } catch (error) {
			reject(error);
		    }
		};

		const onResponseBound = onResponse.bind(this);

		// Listen for response messages until we receive the available topics
		const messageListener = (event: MessageEvent) => {
		    if (event.data instanceof Blob) {
			const reader = new FileReader();
			reader.onload = () => {
			    const text = reader.result as string;
			    onResponseBound(text);
			    this.ws?.removeEventListener('message', messageListener);
			};
			reader.onerror = (error) => {
			    reject(new Error('Error reading Blob data: ' + error));
			};
			reader.readAsText(event.data);
		    } else {
			onResponseBound(event.data);
			this.ws?.removeEventListener('message', messageListener);
		    }
		};
		this.ws?.addEventListener('message', messageListener);
	    });
	}

    subscribe(topicName: string, maxUpdateRate: number): void {
        const payload: SubscribePayload = {
            topicName,
            maxUpdateRate
        };
        const message = JSON.stringify(['s', payload]);
	// console.log(message);
	console.log ("Subscribing to ", topicName);
	if (message !== undefined) {
            this.send(message);
	} else {
	    console.error("Message is undefined. Cannot send.");
	}
        this.subscribedTopics.push(topicName);
    }

    unsubscribe(topicName: string): void {
        const payload: UnsubscribePayload = {
            topicName
        };
        const message = JSON.stringify(['u', payload]);
	console.log ("Un-Subscribing to ", topicName);
	if (message !== undefined) {
            this.send(message);
	} else {
	    console.error("Message is undefined. Cannot send.");
	}
        const index = this.subscribedTopics.indexOf(topicName);
        if (index !== -1) {
            this.subscribedTopics.splice(index, 1);
        }
    }

    addTopicCallback(topicName: string, callback: MessageCallback): void {
        this.topicCallbacks[topicName] = callback;
        // Subscribe to the topic when adding the callback
        this.subscribe(topicName, 24);
    }

    private send(message: string): void {
        if (this.ws) {
            this.ws.send(message);
            // console.log('Sent message:', message);
        } else {
            console.error('WebSocket connection is not established');
        }
    }

    close(): void {
        if (this.ws) {
            this.ws.close();
            this.ws = undefined;
            this.closed = true;
            console.log('WebSocket connection closed');
        } else {
            console.warn('WebSocket connection is already closed');
        }
    }
}
