"use client"

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Progress } from "@/components/ui/progress"
import { Separator } from "@/components/ui/separator"
import { useHotelData } from "@/hooks/useHotelData"
import {
  CreditCard,
  Battery,
  BatteryLow,
  Activity,
  CheckCircle,
  AlertTriangle,
  Phone,
  MessageSquare,
  Wifi,
  Car,
  Utensils,
  Dumbbell,
} from "lucide-react"


interface GuestDashboardProps {
  hotelId?: string
}

export function GuestDashboard({ hotelId = "1" }: GuestDashboardProps) {
  const { hotel, rooms, activities, loading, error } = useHotelData(hotelId)
  
  // Mock guest data (fallback)
  const guestCard = {
    id: "KC001",
    roomNumber: "101",
    status: "active",
    batteryLevel: 85,
    lastAccess: "2024-01-15T14:30:00Z",
    accessCount: 12,
    issuedDate: "2024-01-14T10:00:00Z",
    expiryDate: "2024-01-17T12:00:00Z",
    checkInDate: "2024-01-14T15:00:00Z",
    checkOutDate: "2024-01-17T11:00:00Z",
  }

  const recentAccess = [
    { time: "2024-01-15T14:30:00Z", location: "Room 101", action: "Entry" },
    { time: "2024-01-15T12:15:00Z", location: "Elevator", action: "Access" },
    { time: "2024-01-15T09:45:00Z", location: "Pool Area", action: "Entry" },
    { time: "2024-01-15T08:30:00Z", location: "Gym", action: "Access" },
  ]

  const hotelServices = [
    { name: "WiFi", icon: Wifi, status: "Connected", description: "High-speed internet" },
    { name: "Parking", icon: Car, status: "Available", description: "Complimentary valet" },
    { name: "Restaurant", icon: Utensils, status: "Open", description: "24/7 room service" },
    { name: "Fitness", icon: Dumbbell, status: "Open", description: "24/7 gym access" },
  ]

  const formatTime = (dateString: string) => {
    return new Date(dateString).toLocaleTimeString()
  }

  const formatDate = (dateString: string) => {
    return new Date(dateString).toLocaleDateString()
  }

  const getDaysRemaining = () => {
    const checkOut = new Date(guestCard.checkOutDate)
    const now = new Date()
    const diffTime = checkOut.getTime() - now.getTime()
    const diffDays = Math.ceil(diffTime / (1000 * 60 * 60 * 24))
    return Math.max(0, diffDays)
  }

  return (
    <div className="space-y-6 relative">
      {/* Welcome Section */}
      <div>
        <h1 className="text-3xl font-bold">Welcome Back!</h1>
        <p className="text-muted-foreground">Your stay information and key card details</p>
      </div>

      {/* Stay Overview */}
      <Card className="bg-gradient-to-r from-blue-50 to-indigo-50 dark:from-blue-950 dark:to-indigo-950">
        <CardContent className="p-6">
          <div className="flex items-center justify-between mb-4">
            <div>
              <h2 className="text-2xl font-bold">Room {guestCard.roomNumber}</h2>
              <p className="text-muted-foreground">Your current accommodation</p>
            </div>
            <Badge variant="outline" className="bg-green-50 text-green-700 border-green-200">
              <CheckCircle className="h-3 w-3 mr-1" />
              Active Stay
            </Badge>
          </div>

          <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
            <div>
              <p className="text-sm text-muted-foreground">Check-in</p>
              <p className="font-semibold">{formatDate(guestCard.checkInDate)}</p>
            </div>
            <div>
              <p className="text-sm text-muted-foreground">Check-out</p>
              <p className="font-semibold">{formatDate(guestCard.checkOutDate)}</p>
            </div>
            <div>
              <p className="text-sm text-muted-foreground">Days Remaining</p>
              <p className="font-semibold">{getDaysRemaining()} days</p>
            </div>
            <div>
              <p className="text-sm text-muted-foreground">Card Uses</p>
              <p className="font-semibold">{guestCard.accessCount} times</p>
            </div>
          </div>
        </CardContent>
      </Card>

      {/* Key Card Status */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center">
            <CreditCard className="h-5 w-5 mr-2" />
            Your Key Card
          </CardTitle>
          <CardDescription>Card ID: {guestCard.id}</CardDescription>
        </CardHeader>
        <CardContent className="space-y-4">
          <div className="flex items-center justify-between">
            <div className="flex items-center space-x-2">
              <Badge variant="outline" className="bg-green-50 text-green-700 border-green-200">
                <CheckCircle className="h-3 w-3 mr-1" />
                Active
              </Badge>
              <span className="text-sm text-muted-foreground">Last used: {formatTime(guestCard.lastAccess)}</span>
            </div>
          </div>

          <div className="space-y-2">
            <div className="flex items-center justify-between text-sm">
              <span className="flex items-center">
                {guestCard.batteryLevel < 20 ? (
                  <BatteryLow className="h-4 w-4 mr-1 text-red-500" />
                ) : (
                  <Battery className="h-4 w-4 mr-1" />
                )}
                Battery Level
              </span>
              <span className="font-medium">{guestCard.batteryLevel}%</span>
            </div>
            <Progress value={guestCard.batteryLevel} className="h-2" />
            {guestCard.batteryLevel < 20 && (
              <div className="flex items-center text-sm text-red-600">
                <AlertTriangle className="h-4 w-4 mr-1" />
                Low battery - please contact front desk for replacement
              </div>
            )}
          </div>

          <Separator />

          <div className="grid grid-cols-2 gap-4 text-sm">
            <div>
              <p className="text-muted-foreground">Issued</p>
              <p className="font-medium">{formatDate(guestCard.issuedDate)}</p>
            </div>
            <div>
              <p className="text-muted-foreground">Expires</p>
              <p className="font-medium">{formatDate(guestCard.expiryDate)}</p>
            </div>
          </div>
        </CardContent>
      </Card>

      {/* Recent Access History */}
      <Card>
        <CardHeader>
          <CardTitle className="flex items-center">
            <Activity className="h-5 w-5 mr-2" />
            Recent Access History
          </CardTitle>
          <CardDescription>Your recent key card usage</CardDescription>
        </CardHeader>
        <CardContent>
          <div className="space-y-4">
            {recentAccess.map((access, index) => (
              <div key={index} className="flex items-center justify-between p-3 rounded-lg border">
                <div className="flex items-center space-x-3">
                  <div className="w-2 h-2 bg-green-500 rounded-full"></div>
                  <div>
                    <p className="font-medium">{access.location}</p>
                    <p className="text-sm text-muted-foreground">{access.action}</p>
                  </div>
                </div>
                <div className="text-right">
                  <p className="text-sm font-medium">{formatTime(access.time)}</p>
                  <p className="text-xs text-muted-foreground">{formatDate(access.time)}</p>
                </div>
              </div>
            ))}
          </div>
        </CardContent>
      </Card>

      {/* Hotel Services */}
      <Card>
        <CardHeader>
          <CardTitle>Hotel Services</CardTitle>
          <CardDescription>Available amenities and services</CardDescription>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            {hotelServices.map((service, index) => (
              <div key={index} className="flex items-center space-x-3 p-3 rounded-lg border">
                <service.icon className="h-8 w-8 text-blue-500" />
                <div className="flex-1">
                  <div className="flex items-center justify-between">
                    <h3 className="font-medium">{service.name}</h3>
                    <Badge variant="outline" className="text-xs">
                      {service.status}
                    </Badge>
                  </div>
                  <p className="text-sm text-muted-foreground">{service.description}</p>
                </div>
              </div>
            ))}
          </div>
        </CardContent>
      </Card>

      {/* Quick Actions */}
      <Card>
        <CardHeader>
          <CardTitle>Need Assistance?</CardTitle>
          <CardDescription>Contact hotel services</CardDescription>
        </CardHeader>
        <CardContent className="space-y-3">
          <Button className="w-full justify-start bg-transparent" variant="outline">
            <Phone className="h-4 w-4 mr-2" />
            Call Front Desk
          </Button>
          <Button className="w-full justify-start bg-transparent" variant="outline">
            <MessageSquare className="h-4 w-4 mr-2" />
            Request Service
          </Button>
          <Button className="w-full justify-start bg-transparent" variant="outline">
            <CreditCard className="h-4 w-4 mr-2" />
            Report Card Issue
          </Button>
        </CardContent>
      </Card>
    </div>
  )
}
