// API service layer for connecting to live server
/**
 * @file api.ts
 * @brief Typed API client for communicating with the hotel backend.
 */

/**
 * Resolve the backend API base URL.
 * Prefers NEXT_PUBLIC_API_URL when set, otherwise falls back to the
 * deployed Render backend URL.
 */
function getApiBaseUrl(): string {
  const envUrl = process.env.NEXT_PUBLIC_API_URL || 'https://hotel-backend-5kcn.onrender.com';

  if (typeof window !== 'undefined') {
    try {
      const url = new URL(envUrl);
      if (url.hostname === 'localhost') {
        const hostname = window.location.hostname;
        const portPart = url.port ? `:${url.port}` : '';
        return `${url.protocol}//${hostname}${portPart}`;
      }
    } catch (error) {
      // If URL parsing fails, fall back to envUrl
    }
  }

  return envUrl;
}

const API_BASE_URL = getApiBaseUrl();

/**
 * Hotel metadata and derived occupancy information.
 */
export interface Hotel {
  id: string;
  name: string;
  location: string;
  address: string;
  phone: string;
  email: string;
  rating: number;
  description: string;
  image: string;
  status: string;
  lastActivity: string;
  manager: {
    name: string;
    phone: string;
    email: string;
    status: string;
  };
  totalRooms?: number;
  activeRooms?: number;
  occupancy?: number;
}

/**
 * Room state as returned by the backend.
 */
export interface Room {
  hotelId: string;
  id: number;
  number: string;
  status: string;
  hasMasterKey: boolean;
  hasLowPower: boolean;
  powerStatus: string;
  occupantType: string | null;
  lastSeenAt?: string;
}

/**
 * Attendance record for a single check-in/check-out event.
 */
export interface Attendance {
  hotelId: string;
  card_uid: string;
  role: string;
  check_in: string;
  check_out: string;
  duration: number;
  room: string;
}

/**
 * Security or system alert associated with a room or card.
 */
export interface Alert {
  hotelId: string;
  card_uid: string;
  role: string;
  alert_message: string;
  triggered_at: string;
  room: string;
}

/**
 * Record of an unauthorized card access attempt.
 */
export interface DeniedAccess {
  hotelId: string;
  card_uid: string;
  role: string;
  denial_reason: string;
  attempted_at: string;
  room: string;
}

/**
 * High-level activity entry used in the activity log UI.
 */
export interface Activity {
  hotelId: string;
  id: string;
  type: string;
  action: string;
  user: string;
  time: string;
}

/**
 * Hotel staff or system user.
 */
export interface User {
  hotelId: string;
  id: string;
  name: string;
  email: string;
  role: string;
  status: string;
  lastLogin: string;
  avatar: string;
}

/**
 * Guest or staff RFID card assignment.
 */
export interface Card {
  hotelId: string;
  id: string;
  roomNumber: string;
  guestName: string;
  status: string;
  expiryDate: string;
  lastUsed: string;
}

/**
 * Latest power/current reading per room.
 */
export interface PowerReading {
  hotelId: string;
  room: string;
  current: number;
  timestamp: string;
}

/**
 * Per-hotel configurable logic settings.
 */
export interface Settings {
  hotelId: string;
  minCleaningDurationSeconds: number;
  lowPowerCurrentThreshold: number;
}

/**
 * Thin wrapper around fetch for talking to the backend REST API.
 */
class ApiService {
  /**
   * Perform a typed HTTP request against the backend.
   * @param endpoint Relative API path (e.g. "/api/hotel/1").
   * @param options  Fetch options such as method, headers, and body.
   */
  private async request<T>(endpoint: string, options?: RequestInit): Promise<T> {
    const url = `${API_BASE_URL}${endpoint}`;
    
    try {
      const response = await fetch(url, {
        headers: {
          'Content-Type': 'application/json',
          ...options?.headers,
        },
        ...options,
      });

      if (!response.ok) {
        throw new Error(`API request failed: ${response.status} ${response.statusText}`);
      }

      return response.json();
    } catch (error) {
      console.error(`API request failed for ${endpoint}:`, error);
      throw new Error(`Failed to connect to server. Please ensure the backend server is running on ${API_BASE_URL}`);
    }
  }

  // Hotel endpoints
  /** Fetch list of all hotels. */
  async getHotels(): Promise<Hotel[]> {
    return this.request<Hotel[]>('/api/hotels');
  }

  /** Fetch a single hotel with derived occupancy details. */
  async getHotel(hotelId: string): Promise<Hotel> {
    return this.request<Hotel>(`/api/hotel/${hotelId}`);
  }

  /** Update hotel metadata such as name, address, or manager. */
  async updateHotel(hotelId: string, data: Partial<Hotel>): Promise<{ message: string }> {
    return this.request<{ message: string }>(`/api/hotel/${hotelId}`, {
      method: 'PUT',
      body: JSON.stringify(data),
    });
  }

  // Room endpoints
  /** Fetch all rooms for a given hotel. */
  async getRooms(hotelId: string): Promise<Room[]> {
    return this.request<Room[]>(`/api/rooms/${hotelId}`);
  }

  // Attendance endpoints
  /** Fetch attendance history for a hotel. */
  async getAttendance(hotelId: string): Promise<Attendance[]> {
    return this.request<Attendance[]>(`/api/attendance/${hotelId}`);
  }

  // Alert endpoints
  /** Fetch alert history for a hotel. */
  async getAlerts(hotelId: string): Promise<Alert[]> {
    return this.request<Alert[]>(`/api/alerts/${hotelId}`);
  }

  // Denied access endpoints
  /** Fetch denied access attempts for a hotel. */
  async getDeniedAccess(hotelId: string): Promise<DeniedAccess[]> {
    return this.request<DeniedAccess[]>(`/api/denied_access/${hotelId}`);
  }

  // User endpoints
  /** Fetch all users/staff for a hotel. */
  async getUsers(hotelId: string): Promise<User[]> {
    return this.request<User[]>(`/api/users/${hotelId}`);
  }

  // Card endpoints
  /** Fetch card assignments for a hotel. */
  async getCards(hotelId: string): Promise<Card[]> {
    return this.request<Card[]>(`/api/cards/${hotelId}`);
  }

  // Activity endpoints
  /** Fetch recent high-level activity entries for a hotel. */
  async getActivity(hotelId: string): Promise<Activity[]> {
    return this.request<Activity[]>(`/api/activity/${hotelId}`);
  }

  // Settings endpoints
  /** Fetch effective settings for a hotel. */
  async getSettings(hotelId: string): Promise<Settings> {
    return this.request<Settings>(`/api/settings/${hotelId}`);
  }

  /** Update per-hotel logic settings (cleaning duration, low power threshold). */
  async updateSettings(hotelId: string, data: Partial<Settings>): Promise<Settings> {
    return this.request<Settings>(`/api/settings/${hotelId}`, {
      method: 'PUT',
      body: JSON.stringify(data),
    });
  }

  // Power endpoints
  /** Fetch latest power readings for a hotel. */
  async getPower(hotelId: string): Promise<PowerReading[]> {
    return this.request<PowerReading[]>(`/api/power/${hotelId}`);
  }
}

export const apiService = new ApiService();
