// Socket.IO client service for real-time updates
/**
 * @file socket.ts
 * @brief Real-time client for receiving room and activity updates via
 *        WebSocket with automatic fallback to Server-Sent Events (SSE).
 */

/**
 * Resolve the base URL used for WebSocket/SSE connections.
 * Prefers NEXT_PUBLIC_SOCKET_URL, otherwise falls back to the
 * deployed Render backend URL.
 */
function getSocketBaseUrl(): string {
  return process.env.NEXT_PUBLIC_SOCKET_URL || 'https://hotel-backend-5kcn.onrender.com';
}

const SOCKET_URL = getSocketBaseUrl();
const IS_PRODUCTION = process.env.NODE_ENV === 'production';

/**
 * Manage a single logical socket connection that can use either WebSocket
 * or SSE under the hood, with reconnection and per-hotel listeners.
 */
class SocketService {
  private socket: any = null;
  private isConnected = false;
  private eventListeners: Map<string, Function[]> = new Map();
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 2; // Reduced from 5 to 2 for faster fallback
  private reconnectInterval = 1000; // Reduced from 3000ms to 1000ms
  private reconnectTimer: any = null;
  private fallbackToPolling = false;
  private pollingInterval: any = null;
  private sseConnections: Map<string, EventSource> = new Map();
  private useSSE = false;

  /**
   * Establish a connection if needed.
   *
   * In production, this goes directly to SSE. In development, it will
   * try WebSocket first with a quick timeout, then fall back to SSE.
   */
  connect(): any {
    if (this.socket && this.isConnected) {
      return this.socket;
    }

    // In production, skip WebSocket and go directly to SSE
    if (IS_PRODUCTION && !this.useSSE) {
      console.log('Using SSE directly for production environment');
      this.switchToSSE();
      return null;
    }

    // Try WebSocket first, fallback to SSE if it fails (for non-Render, non-localhost hosts)
    if (!this.useSSE) {
      this.connectWebSocket();
    } else {
      console.log('Using SSE fallback mode');
    }

    return this.socket;
  }

  /**
   * Attempt to open a WebSocket connection to the backend /ws endpoint.
   * On timeout or error, it will switch to SSE.
   */
  private connectWebSocket(): void {
    try {
      const wsUrl = SOCKET_URL.replace('https://', 'wss://').replace('http://', 'ws://') + '/ws';
      console.log('Attempting WebSocket connection to:', wsUrl);
      this.socket = new WebSocket(wsUrl);
      
      // Set connection timeout - much faster for immediate fallback
      const connectionTimeout = setTimeout(() => {
        if (this.socket && this.socket.readyState === WebSocket.CONNECTING) {
          console.log('WebSocket connection timeout, switching to SSE');
          this.socket.close();
          this.switchToSSE();
        }
      }, 3000); // Reduced from 10s to 3s timeout
      
      this.socket.onopen = () => {
        console.log('✅ Connected to WebSocket server');
        this.isConnected = true;
        this.reconnectAttempts = 0;
        this.useSSE = false;
        clearTimeout(connectionTimeout);
        if (this.reconnectTimer) {
          clearTimeout(this.reconnectTimer);
          this.reconnectTimer = null;
        }
      };

      this.socket.onclose = (event: CloseEvent) => {
        console.log('❌ WebSocket connection closed:', event.code, event.reason);
        this.isConnected = false;
        clearTimeout(connectionTimeout);
        
        // If it's error 1006 (connection blocked), switch to SSE immediately
        if (event.code === 1006) {
          console.log('WebSocket blocked (1006), switching to SSE immediately');
          this.switchToSSE();
        } else {
          this.attemptReconnect();
        }
      };

      this.socket.onerror = (event: Event | any) => {
        const message =
          event && typeof event === 'object'
            ? (event.message || event.type || 'Unknown WebSocket error')
            : String(event ?? 'Unknown WebSocket error');

        console.warn('🚨 WebSocket connection error:', message);
        this.isConnected = false;
        clearTimeout(connectionTimeout);

        // On any connection error, immediately switch to SSE fallback
        if (!this.useSSE) {
          console.log('WebSocket error, switching to SSE fallback');
          this.switchToSSE();
        }
      };

      this.socket.onmessage = (event: MessageEvent) => {
        try {
          const data = JSON.parse(event.data);
          const eventName = data.event || data.type;
          const listeners = this.eventListeners.get(eventName);
          if (listeners) {
            listeners.forEach(callback => callback(data.data || data));
          }
        } catch (error) {
          console.error('Error parsing socket message:', error);
        }
      };
    } catch (error) {
      console.error('Failed to create WebSocket connection:', error);
      this.isConnected = false;
    }

    return this.socket;
  }

  /**
   * Try to reconnect the WebSocket a limited number of times before
   * switching to SSE fallback.
   */
  private attemptReconnect(): void {
    if (this.useSSE) {
      return; // Don't reconnect WebSocket if using SSE
    }

    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.log('Max WebSocket reconnection attempts reached. Switching to SSE fallback.');
      this.switchToSSE();
      return;
    }

    this.reconnectAttempts++;
    console.log(`Attempting WebSocket reconnect... (${this.reconnectAttempts}/${this.maxReconnectAttempts})`);
    
    this.reconnectTimer = setTimeout(() => {
      this.connectWebSocket();
    }, this.reconnectInterval); // Fixed interval for faster reconnection
  }

  /**
   * Open an SSE connection for a specific hotel, listening to
   * /api/events/:hotelId and forwarding events to listeners.
   */
  private connectSSE(hotelId: string): void {
    if (this.sseConnections.has(hotelId)) {
      return; // Already connected for this hotel
    }

    const sseUrl = `${SOCKET_URL}/api/events/${hotelId}`;
    console.log(`Connecting to SSE for hotel ${hotelId}:`, sseUrl);
    
    try {
      const eventSource = new EventSource(sseUrl);
      
      eventSource.onopen = () => {
        console.log(`✅ SSE connected for hotel ${hotelId}`);
        this.isConnected = true;
      };
      
      eventSource.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          const eventName = data.event || data.type;
          const listeners = this.eventListeners.get(eventName);
          if (listeners) {
            listeners.forEach(callback => callback(data.data || data));
          }
        } catch (error) {
          console.error('Error parsing SSE message:', error);
        }
      };
      
      eventSource.onerror = (event: any) => {
        const message =
          event && typeof event === 'object'
            ? (event.message || event.type || 'Unknown SSE error')
            : String(event ?? 'Unknown SSE error');

        console.warn(`SSE connection issue for hotel ${hotelId}:`, message);
        this.sseConnections.delete(hotelId);

        // If the stream has been explicitly closed, don't try to reconnect
        if (eventSource.readyState === 2 /* CLOSED */) {
          console.log(`SSE connection closed for hotel ${hotelId}`);
          return;
        }

        // Attempt to reconnect SSE after a delay with exponential backoff
        const retryDelay = Math.min(5000 * Math.pow(2, this.reconnectAttempts), 30000);
        setTimeout(() => {
          if (this.useSSE && !this.sseConnections.has(hotelId)) {
            this.reconnectAttempts++;
            if (this.reconnectAttempts < this.maxReconnectAttempts) {
              this.connectSSE(hotelId);
            } else {
              console.log(`Max SSE reconnection attempts reached for hotel ${hotelId}`);
            }
          }
        }, retryDelay);
      };
      
      this.sseConnections.set(hotelId, eventSource);
    } catch (error) {
      console.error(`Failed to create SSE connection for hotel ${hotelId}:`, error);
    }
  }

  /**
   * Cleanly tear down any WebSocket connection and mark the client
   * as using SSE for future subscriptions.
   */
  private switchToSSE(): void {
    console.log('🔄 Switching to Server-Sent Events (SSE) fallback');
    this.useSSE = true;
    this.isConnected = false;
    this.reconnectAttempts = 0;
    
    // Clean up WebSocket
    if (this.socket) {
      this.socket.close();
      this.socket = null;
    }
    
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    
    // SSE connections will be established per hotel when listeners are added
  }

  /**
   * Disconnect from all transports and clear timers/listeners.
   */
  disconnect(): void {
    console.log('🔌 Disconnecting from socket service');
    
    // Clear reconnection timer
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    
    // Clear polling interval
    if (this.pollingInterval) {
      clearInterval(this.pollingInterval);
      this.pollingInterval = null;
    }
    
    // Close WebSocket connection
    if (this.socket) {
      this.socket.close();
      this.socket = null;
    }
    
    // Close all SSE connections
    this.sseConnections.forEach((eventSource, hotelId) => {
      console.log(`Closing SSE connection for hotel ${hotelId}`);
      try {
        eventSource.close();
      } catch (error) {
        console.error(`Error closing SSE connection for hotel ${hotelId}:`, error);
      }
    });
    this.sseConnections.clear();
    
    this.isConnected = false;
    this.useSSE = false;
    this.reconnectAttempts = 0;
    this.eventListeners.clear();
  }

  // Room update listeners
  /**
   * Subscribe to roomUpdate events for a specific hotel.
   */
  onRoomUpdate(hotelId: string, callback: (data: any) => void): void {
    const eventName = `roomUpdate:${hotelId}`;
    if (!this.eventListeners.has(eventName)) {
      this.eventListeners.set(eventName, []);
    }
    this.eventListeners.get(eventName)!.push(callback);
    
    // Connect using appropriate method
    if (this.useSSE) {
      this.connectSSE(hotelId);
    } else if (!this.socket) {
      this.connect();
    }
  }

  /**
   * Unsubscribe a specific callback from roomUpdate events.
   */
  offRoomUpdate(hotelId: string, callback?: (data: any) => void): void {
    const eventName = `roomUpdate:${hotelId}`;
    if (callback && this.eventListeners.has(eventName)) {
      const listeners = this.eventListeners.get(eventName)!;
      const index = listeners.indexOf(callback);
      if (index > -1) {
        listeners.splice(index, 1);
      }
    }
  }

  // Activity update listeners
  /**
   * Subscribe to activityUpdate events for a specific hotel.
   */
  onActivityUpdate(hotelId: string, callback: (data: any) => void): void {
    const eventName = `activityUpdate:${hotelId}`;
    if (!this.eventListeners.has(eventName)) {
      this.eventListeners.set(eventName, []);
    }
    this.eventListeners.get(eventName)!.push(callback);
    
    // Connect using appropriate method
    if (this.useSSE) {
      this.connectSSE(hotelId);
    } else if (!this.socket) {
      this.connect();
    }
  }

  /**
   * Unsubscribe a specific callback from activityUpdate events.
   */
  offActivityUpdate(hotelId: string, callback?: (data: any) => void): void {
    const eventName = `activityUpdate:${hotelId}`;
    if (callback && this.eventListeners.has(eventName)) {
      const listeners = this.eventListeners.get(eventName)!;
      const index = listeners.indexOf(callback);
      if (index > -1) {
        listeners.splice(index, 1);
      }
    }
  }

  // Generic event listeners
  /**
   * Subscribe to a custom event coming through the socket channel.
   */
  on(event: string, callback: (...args: any[]) => void): void {
    if (!this.socket) {
      this.connect();
    }
    
    if (!this.eventListeners.has(event)) {
      this.eventListeners.set(event, []);
    }
    this.eventListeners.get(event)!.push(callback);
  }

  /**
   * Unsubscribe a specific callback from a custom event.
   */
  off(event: string, callback?: (...args: any[]) => void): void {
    if (callback && this.eventListeners.has(event)) {
      const listeners = this.eventListeners.get(event)!;
      const index = listeners.indexOf(callback);
      if (index > -1) {
        listeners.splice(index, 1);
      }
    }
  }

  // Emit events
  /**
   * Emit an event payload over the WebSocket connection (if active).
   */
  emit(event: string, ...args: any[]): void {
    if (this.socket && this.isConnected) {
      this.socket.send(JSON.stringify({
        event,
        data: args.length === 1 ? args[0] : args
      }));
    }
  }

  /** Get raw underlying WebSocket instance (if any). */
  getSocket(): any {
    return this.socket;
  }

  /** Return true if the socket layer is currently connected. */
  isSocketConnected(): boolean {
    return this.isConnected;
  }
}

export const socketService = new SocketService();
