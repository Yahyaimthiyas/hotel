/**
 * @file README.md
 * @brief RFID Hotel Management System - Complete Documentation
 * @author Development Team
 * @version 1.0.0
 * @date October 2025
 * 
 * @mainpage RFID Hotel Management System
 * 
 * A comprehensive, production-ready hotel management system with RFID key card integration,
 * real-time monitoring, and advanced analytics dashboard. This system manages 8 hotels
 * across Tamil Nadu and Karnataka with complete IoT integration.
 * 
 * @section features Key Features
 * - Multi-hotel management with centralized dashboard
 * - ESP32-based RFID key card system with real-time tracking
 * - Advanced analytics with 8 comprehensive dashboard sections
 * - Role-based access control (Administrator/Staff/Guest)
 * - Mobile-first responsive design with glassmorphism UI
 * - Real-time communication via WebSocket, MQTT, and SSE
 * - Production deployment on Vercel, Render, and MongoDB Atlas
 * 
 * @section architecture System Architecture
 * - **Frontend**: Next.js 15 with TypeScript and Tailwind CSS
 * - **Backend**: Node.js/Express with MongoDB and MQTT broker
 * - **Hardware**: ESP32 with MFRC522 RFID readers
 * - **Real-time**: WebSocket + MQTT + Server-Sent Events
 * - **Deployment**: Vercel (Frontend) + Render (Backend) + MongoDB Atlas
 */

# 🏨 RFID Hotel Management System

[![Deploy Status](https://img.shields.io/badge/Deploy-Production%20Ready-brightgreen)](https://hotel-frontend-two-puce.vercel.app)
[![Backend](https://img.shields.io/badge/Backend-Online-success)](https://hotel-backend-5kcn.onrender.com)
[![Database](https://img.shields.io/badge/Database-MongoDB%20Atlas-green)](https://cloud.mongodb.com)
[![Hardware](https://img.shields.io/badge/Hardware-ESP32%20Ready-blue)](#esp32-setup)

> **🚀 Live Demo**: [https://hotel-frontend-two-puce.vercel.app](https://hotel-frontend-two-puce.vercel.app)  
> **📡 Backend API**: [https://hotel-backend-5kcn.onrender.com](https://hotel-backend-5kcn.onrender.com)

## 📋 Table of Contents

- [🎯 Overview](#overview)
- [✨ Features](#features)
- [🏗️ Architecture](#architecture)
- [🚀 Quick Start](#quick-start)
- [🔧 Installation](#installation)
- [🌐 Deployment](#deployment)
- [🔌 Hardware Setup](#hardware-setup)
- [📊 Analytics](#analytics)
- [🔐 Authentication](#authentication)
- [📱 Mobile Support](#mobile-support)
- [🧪 Testing](#testing)
- [📚 API Documentation](#api-documentation)
- [🤝 Contributing](#contributing)
- [📄 License](#license)

---

## 🎯 Overview

The **RFID Hotel Management System** is a modern, scalable solution for hotel operations management. It combines cutting-edge web technologies with IoT hardware to provide real-time room monitoring, guest management, and comprehensive analytics.

### **🎪 Demo Accounts**

| Role | Email | Password | Access Level |
|------|-------|----------|--------------|
| **Administrator** | `admin@hotel.com` | `admin123` | Full system access, multi-hotel management |
| **Staff** | `staff@hotel.com` | `staff123` | Hotel operations, guest management |
| **Guest** | `guest@hotel.com` | `guest123` | Personal key card information |

---

## ✨ Features

### 🏨 **Multi-Hotel Management**
- **8 Pre-configured Hotels** across Tamil Nadu and Karnataka
- **Interactive Hotel Cards** with real-time metrics
- **Centralized Dashboard** for administrators
- **Hotel-specific Management** interfaces

### 🔐 **RFID Key Card System**
- **ESP32-based Hardware** integration
- **Real-time Card Tracking** with MQTT protocol
- **Automatic Check-in/Check-out** detection
- **Security Alerts** for unauthorized access
- **Card Lifecycle Management** (issue, activate, expire)

### 📊 **Advanced Analytics**
- **8 Comprehensive Dashboards**:
  - 📈 Overview & Performance
  - ⚡ Real-time Monitoring
  - 🏠 Occupancy Analytics
  - 💰 Revenue Tracking
  - 👥 Guest Demographics
  - 🛎️ Service Quality
  - 📋 Activity Patterns
- **Mobile-Responsive Charts** using Recharts
- **Export Functionality** for reports

### 🔄 **Real-time Features**
- **WebSocket Communication** with SSE fallback
- **Live Room Status** updates
- **Activity Feed** with real-time notifications
- **System Monitoring** dashboard
- **Multi-client Synchronization**

### 📱 **Mobile-First Design**
- **Responsive UI** across all devices
- **Touch-optimized** interactions
- **Progressive Web App** capabilities
- **Glassmorphism Design** with modern aesthetics
- **8-breakpoint Responsive System**

### 🛡️ **Security & Authentication**
- **Role-based Access Control** (Admin/Staff/Guest)
- **JWT Authentication** with secure sessions
- **Environment-based Configuration**
- **CORS Protection** and rate limiting
- **Input Validation** and sanitization

---

## 🏗️ Architecture

```mermaid
graph TB
    subgraph "Frontend (Vercel)"
        A[Next.js 15 App]
        B[React Components]
        C[Tailwind CSS]
        D[shadcn/ui]
    end
    
    subgraph "Backend (Render)"
        E[Express.js Server]
        F[WebSocket Server]
        G[MQTT Broker]
        H[REST API]
    end
    
    subgraph "Database (MongoDB Atlas)"
        I[Hotels Collection]
        J[Rooms Collection]
        K[Activities Collection]
        L[Users Collection]
    end
    
    subgraph "Hardware (ESP32)"
        M[RFID Reader]
        N[WiFi Module]
        O[MQTT Client]
    end
    
    A --> E
    E --> I
    M --> G
    G --> F
    F --> A
    
    style A fill:#e1f5fe
    style E fill:#f3e5f5
    style I fill:#e8f5e8
    style M fill:#fff3e0
```

### **Technology Stack**

#### **Frontend**
- **Framework**: Next.js 15 with App Router
- **Language**: TypeScript
- **Styling**: Tailwind CSS + Custom Design System
- **UI Library**: shadcn/ui + Radix UI
- **Charts**: Recharts (responsive)
- **State**: React Context + Hooks
- **Deployment**: Vercel

#### **Backend**
- **Runtime**: Node.js 18+
- **Framework**: Express.js
- **Database**: MongoDB with Mongoose ODM
- **Real-time**: WebSocket + MQTT (Aedes)
- **Authentication**: JWT + Sessions
- **Deployment**: Render.com

#### **Hardware**
- **Microcontroller**: ESP32
- **RFID**: MFRC522 Reader
- **Protocol**: MQTT over WebSocket
- **Connectivity**: WiFi

---

## 🚀 Quick Start

### **1. Clone Repository**
```bash
git clone https://github.com/your-username/rfid-hotel-management.git
cd rfid-hotel-management
```

### **2. Install Dependencies**
```bash
# Frontend
npm install

# Backend
cd Backend
npm install
```

### **3. Environment Setup**
```bash
# Frontend (.env.local)
NEXT_PUBLIC_API_URL=https://hotel-backend-5kcn.onrender.com
NEXT_PUBLIC_SOCKET_URL=https://hotel-backend-5kcn.onrender.com

# Backend (.env)
MONGO_URL=your-mongodb-connection-string
PORT=3000
FRONTEND_URL=http://localhost:3000
```

### **4. Run Development**
```bash
# Backend (Terminal 1)
cd Backend
npm start

# Frontend (Terminal 2)
npm run dev
```

### **5. Access Application**
- **Frontend**: http://localhost:3000
- **Backend API**: http://localhost:3000/api
- **Login**: Use demo accounts above

---

## 🔧 Installation

### **Prerequisites**
- Node.js 18+ (LTS recommended)
- npm/yarn/pnpm package manager
- MongoDB Atlas account
- Git version control

### **Detailed Setup**

#### **1. Frontend Setup**
```bash
# Install dependencies
npm install

# Install additional packages if needed
npm install @radix-ui/react-* lucide-react recharts

# Development server
npm run dev

# Production build
npm run build
npm start
```

#### **2. Backend Setup**
```bash
cd Backend

# Install dependencies
npm install

# Required packages
npm install express mongoose cors ws aedes socket.io

# Start server
npm start
```

#### **3. Database Setup**
1. Create MongoDB Atlas cluster
2. Create database user
3. Whitelist IP addresses
4. Copy connection string to `.env`

---

## 🌐 Deployment

### **Production Deployment**

#### **Frontend (Vercel)**
```bash
# Install Vercel CLI
npm i -g vercel

# Deploy
vercel --prod

# Or connect GitHub repository to Vercel
```

#### **Backend (Render)**
1. Connect GitHub repository
2. Configure build settings:
   - **Build Command**: `npm install`
   - **Start Command**: `node index.js`
3. Add environment variables
4. Deploy

#### **Environment Variables**

**Frontend (Vercel)**:
```env
NEXT_PUBLIC_API_URL=https://hotel-backend-5kcn.onrender.com
NEXT_PUBLIC_SOCKET_URL=https://hotel-backend-5kcn.onrender.com
NEXTAUTH_SECRET=your-secret-key
```

**Backend (Render)**:
```env
MONGO_URL=mongodb+srv://user:pass@cluster.mongodb.net/hotel_db
PORT=3000
NODE_ENV=production
FRONTEND_URL=https://hotel-frontend-two-puce.vercel.app
```

---

## 🔌 Hardware Setup

### **ESP32 Configuration**

#### **1. Hardware Connections**
```cpp
// Pin Configuration
#define RST_PIN 22    // Reset pin
#define SS_PIN 21     // Slave Select pin

// MFRC522 to ESP32 connections:
// RST     -> GPIO 22
// SDA(SS) -> GPIO 21  
// MOSI    -> GPIO 23
// MISO    -> GPIO 19
// SCK     -> GPIO 18
// 3.3V    -> 3.3V
// GND     -> GND
```

#### **2. Software Configuration**
```cpp
// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Server configuration
const char* websocketHost = "hotel-backend-5kcn.onrender.com";
const int websocketPort = 443;
const char* websocketPath = "/mqtt";

// Room identification
const char* roomNumber = "202";     // Change per room
const char* floorNumber = "3";      // Hotel ID (1-8)
```

#### **3. RFID Card Setup**
```cpp
// Add your RFID cards
UserAuth users[] = {
  {{0xAF, 0x4D, 0x99, 0x1F}, "Maintenance"},
  {{0xBF, 0xD1, 0x07, 0x1F}, "Manager"},
  {{0xB2, 0xF9, 0x7C, 0x00}, "Guest"}
  // Add more cards here
};
```

#### **4. Upload Code**
1. Install Arduino IDE
2. Install required libraries:
   - MFRC522
   - WebSocketsClient
   - ArduinoJson
3. Upload `ESP32 code/esp32code.cpp`
4. Monitor serial output

---

## 📊 Analytics

### **Dashboard Features**

#### **📈 Overview Dashboard**
- Occupancy trends (line charts)
- Room type distribution (pie charts)
- Key performance indicators
- Real-time metrics

#### **⚡ Real-time Monitoring**
- Live activity feed
- System status indicators
- Active guest tracking
- Alert notifications

#### **🏠 Occupancy Analytics**
- Room-by-room status
- Occupancy patterns
- Peak time analysis
- Efficiency metrics

#### **💰 Revenue Tracking**
- Revenue breakdown by source
- Monthly/yearly trends
- RevPAR and ADR calculations
- Profit margin analysis

#### **👥 Guest Demographics**
- Guest type distribution
- Booking patterns
- Satisfaction ratings
- Repeat guest analysis

#### **🛎️ Service Quality**
- Service ratings (radar charts)
- Response time metrics
- Resolution rates
- Guest feedback analysis

#### **📋 Activity Patterns**
- Daily activity charts
- Check-in/out patterns
- Service request trends
- Peak activity times

### **Chart Types**
- **Line Charts**: Trends over time
- **Bar Charts**: Comparative data
- **Area Charts**: Cumulative metrics
- **Pie Charts**: Distribution data
- **Radar Charts**: Multi-dimensional ratings
- **Composed Charts**: Multiple data series

---

## 🔐 Authentication

### **User Roles**

#### **Administrator**
- **Access**: All hotels and features
- **Permissions**:
  - Hotel management
  - User management
  - System configuration
  - Analytics access
  - Security monitoring

#### **Staff**
- **Access**: Assigned hotel only
- **Permissions**:
  - Room management
  - Guest check-in/out
  - Key card operations
  - Basic analytics
  - Activity logging

#### **Guest**
- **Access**: Personal information only
- **Permissions**:
  - View own key card status
  - Check personal activity
  - Update profile information

### **Security Features**
- **JWT Tokens**: Secure session management
- **Role Validation**: Route-level protection
- **Input Sanitization**: XSS prevention
- **Rate Limiting**: DDoS protection
- **CORS Configuration**: Cross-origin security

---

## 📱 Mobile Support

### **Responsive Design**
- **8 Breakpoints**: xs, sm, md, lg, xl, 2xl, 3xl, 4xl
- **Device Detection**: Mobile, tablet, desktop
- **Orientation Support**: Portrait and landscape
- **Touch Optimization**: 44px minimum touch targets

### **Mobile Features**
- **Floating Navigation**: Auto-hide on scroll
- **Swipe Gestures**: Touch-friendly interactions
- **Responsive Charts**: All analytics work on mobile
- **Progressive Web App**: Installable on devices
- **Offline Support**: Basic functionality without internet

### **Performance**
- **Code Splitting**: Lazy loading components
- **Image Optimization**: WebP and AVIF support
- **Caching Strategy**: Service worker implementation
- **Bundle Size**: Optimized for mobile networks

---

## 🧪 Testing

### **Automated Testing**
```bash
# Run tests
npm test

# Run with coverage
npm run test:coverage

# E2E testing
npm run test:e2e
```

### **Manual Testing**

#### **Frontend Testing**
1. **Authentication Flow**
   - Login with demo accounts
   - Role-based access verification
   - Session management

2. **Dashboard Functionality**
   - Hotel selection and navigation
   - Real-time updates
   - Analytics rendering

3. **Responsive Design**
   - Mobile device testing
   - Tablet orientation changes
   - Desktop scaling

#### **Backend Testing**
```bash
# API endpoint testing
curl https://hotel-backend-5kcn.onrender.com/api/hotels
curl https://hotel-backend-5kcn.onrender.com/health

# WebSocket testing
node test-websocket.js

# Database testing
node test-mongodb.js
```

#### **Hardware Testing**
1. **ESP32 Connection**
   - WiFi connectivity
   - Server communication
   - MQTT message flow

2. **RFID Functionality**
   - Card detection
   - Data transmission
   - Real-time updates

---

## 📚 API Documentation

### **Base URL**
```
Production: https://hotel-backend-5kcn.onrender.com
Development: http://localhost:3000
```

### **Authentication**
```http
POST /api/auth/login
Content-Type: application/json

{
  "email": "admin@hotel.com",
  "password": "admin123"
}
```

### **Hotels**
```http
# Get all hotels
GET /api/hotels

# Get specific hotel
GET /api/hotel/:hotelId

# Update hotel
PUT /api/hotel/:hotelId
```

### **Rooms**
```http
# Get hotel rooms
GET /api/rooms/:hotelId

# Update room status
PUT /api/rooms/:hotelId/:roomId
```

### **Real-time**
```javascript
// WebSocket connection
const ws = new WebSocket('wss://hotel-backend-5kcn.onrender.com/ws');

// Server-Sent Events
const eventSource = new EventSource('/api/events/:hotelId');
```

### **MQTT Topics**
```
# Room attendance
campus/room/{building}/{floor}/{roomNum}/attendance

# Security alerts  
campus/room/{building}/{floor}/{roomNum}/alerts

# Access denied
campus/room/{building}/{floor}/{roomNum}/denied_access
```

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

### **Third-party Licenses**
- **Next.js**: MIT License
- **React**: MIT License
- **Tailwind CSS**: MIT License
- **MongoDB**: Server Side Public License
- **ESP32**: Apache License 2.0

---

## 🏆 Acknowledgments

- **shadcn/ui** for the beautiful component library
- **Vercel** for seamless frontend deployment
- **Render** for reliable backend hosting
- **MongoDB Atlas** for cloud database services
- **ESP32 Community** for hardware support

---

**Made with ❤️ by the Development Team**

**Last Updated**: October 2025 
**Version**: 1.0.0  
**Status**: Production Ready ✅

---

## 🔗 Quick Links

- 🌐 **Live Demo**: [https://hotel-frontend-two-puce.vercel.app](https://hotel-frontend-two-puce.vercel.app)
- 📡 **Backend API**: [https://hotel-backend-5kcn.onrender.com](https://hotel-backend-5kcn.onrender.com)
- 📊 **System Status**: [Status Page](https://hotel-backend-5kcn.onrender.com/health)
- 📚 **API Docs**: [API Documentation](https://hotel-backend-5kcn.onrender.com/api)
- 🔧 **Deployment**: [Deployment Guide](DEPLOYMENT_GUIDE.md)
