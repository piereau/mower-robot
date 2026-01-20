/**
 * WebSocket client for robot telemetry.
 * 
 * Handles connection, reconnection, and message parsing.
 */

import type { RobotTelemetry } from '../types/telemetry';

export type MessageHandler = (telemetry: RobotTelemetry) => void;
export type ConnectionHandler = (connected: boolean) => void;

interface WSClientOptions {
  url: string;
  onMessage: MessageHandler;
  onConnectionChange: ConnectionHandler;
  reconnectInterval?: number;
  maxReconnectAttempts?: number;
}

const DEFAULT_RECONNECT_INTERVAL = 2000;
const MAX_RECONNECT_ATTEMPTS = 10;

export class WSClient {
  private ws: WebSocket | null = null;
  private url: string;
  private onMessage: MessageHandler;
  private onConnectionChange: ConnectionHandler;
  private reconnectInterval: number;
  private maxReconnectAttempts: number;
  private reconnectAttempts = 0;
  private reconnectTimeout: ReturnType<typeof setTimeout> | null = null;
  private intentionalClose = false;

  constructor(options: WSClientOptions) {
    this.url = options.url;
    this.onMessage = options.onMessage;
    this.onConnectionChange = options.onConnectionChange;
    this.reconnectInterval = options.reconnectInterval ?? DEFAULT_RECONNECT_INTERVAL;
    this.maxReconnectAttempts = options.maxReconnectAttempts ?? MAX_RECONNECT_ATTEMPTS;
  }

  /**
   * Establish WebSocket connection
   */
  connect(): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      return;
    }

    this.intentionalClose = false;
    
    try {
      this.ws = new WebSocket(this.url);
      this.setupEventHandlers();
    } catch (error) {
      console.error('[WSClient] Failed to create WebSocket:', error);
      this.scheduleReconnect();
    }
  }

  /**
   * Close WebSocket connection
   */
  disconnect(): void {
    this.intentionalClose = true;
    this.clearReconnectTimeout();
    
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
  }

  /**
   * Send a ping to check connection health
   */
  ping(): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send('ping');
    }
  }

  /**
   * Send a JSON payload over the WebSocket.
   */
  sendJSON(payload: Record<string, unknown>): boolean {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(payload));
      return true;
    }

    return false;
  }

  private setupEventHandlers(): void {
    if (!this.ws) return;

    this.ws.onopen = () => {
      console.log('[WSClient] Connected');
      this.reconnectAttempts = 0;
      this.onConnectionChange(true);
    };

    this.ws.onclose = () => {
      console.log('[WSClient] Disconnected');
      this.onConnectionChange(false);
      
      if (!this.intentionalClose) {
        this.scheduleReconnect();
      }
    };

    this.ws.onerror = (error) => {
      console.error('[WSClient] Error:', error);
    };

    this.ws.onmessage = (event) => {
      try {
        // Ignore pong responses
        if (event.data === 'pong') {
          return;
        }
        
        const telemetry = JSON.parse(event.data) as RobotTelemetry;
        this.onMessage(telemetry);
      } catch (error) {
        console.error('[WSClient] Failed to parse message:', error);
      }
    };
  }

  private scheduleReconnect(): void {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.error('[WSClient] Max reconnect attempts reached');
      return;
    }

    this.clearReconnectTimeout();
    
    // Exponential backoff
    const delay = this.reconnectInterval * Math.pow(1.5, this.reconnectAttempts);
    
    console.log(`[WSClient] Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts + 1})`);
    
    this.reconnectTimeout = setTimeout(() => {
      this.reconnectAttempts++;
      this.connect();
    }, delay);
  }

  private clearReconnectTimeout(): void {
    if (this.reconnectTimeout) {
      clearTimeout(this.reconnectTimeout);
      this.reconnectTimeout = null;
    }
  }
}

/**
 * Create a WebSocket client with default URL from environment
 */
export function createWSClient(
  onMessage: MessageHandler,
  onConnectionChange: ConnectionHandler
): WSClient {
  const url = import.meta.env.VITE_WS_URL || 'ws://localhost:8000/ws/robot';
  
  return new WSClient({
    url,
    onMessage,
    onConnectionChange,
  });
}

