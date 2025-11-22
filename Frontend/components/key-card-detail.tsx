"use client"

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Avatar, AvatarFallback, AvatarImage } from "@/components/ui/avatar"
import { Progress } from "@/components/ui/progress"
import { Separator } from "@/components/ui/separator"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import {
  ArrowLeft,
  MapPin,
  Calendar,
  Clock,
  Battery,
  BatteryLow,
  Activity,
  AlertTriangle,
  CheckCircle,
  XCircle,
  Wifi,
  WifiOff,
  RefreshCw,
  Trash2,
  Edit,
} from "lucide-react"

interface KeyCard {
  id: string
  roomNumber: string
  guestName: string
  status: string
  batteryLevel: number
  lastAccess: string
  accessCount: number
  issuedDate: string
  expiryDate: string
  guestAvatar: string
  accessHistory: Array<{
    time: string
    location: string
    action: string
  }>
}

interface KeyCardDetailProps {
  card: KeyCard
  onBack: () => void
}

const getStatusColor = (status: string) => {
  switch (status) {
    case "active":
      return "bg-green-500"
    case "inactive":
      return "bg-yellow-500"
    case "expired":
      return "bg-red-500"
    default:
      return "bg-gray-500"
  }
}

const getStatusIcon = (status: string) => {
  switch (status) {
    case "active":
      return <CheckCircle className="h-4 w-4" />
    case "inactive":
      return <AlertTriangle className="h-4 w-4" />
    case "expired":
      return <XCircle className="h-4 w-4" />
    default:
      return <Clock className="h-4 w-4" />
  }
}

const getBatteryIcon = (level: number) => {
  return level < 20 ? <BatteryLow className="h-4 w-4 text-red-500" /> : <Battery className="h-4 w-4" />
}

export function KeyCardDetail({ card, onBack }: KeyCardDetailProps) {
  const formatDate = (dateString: string) => {
    return new Date(dateString).toLocaleString()
  }

  const formatTime = (dateString: string) => {
    return new Date(dateString).toLocaleTimeString()
  }

  const isExpiringSoon = () => {
    const expiryDate = new Date(card.expiryDate)
    const now = new Date()
    const hoursUntilExpiry = (expiryDate.getTime() - now.getTime()) / (1000 * 60 * 60)
    return hoursUntilExpiry < 24 && hoursUntilExpiry > 0
  }

  return (
    <div className="min-h-screen bg-background">
      {/* Header */}
      <div className="sticky top-0 z-50 bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60 border-b">
        <div className="container mx-auto px-4 py-4">
          <div className="flex items-center space-x-4">
            <Button variant="ghost" size="sm" onClick={onBack}>
              <ArrowLeft className="h-4 w-4 mr-2" />
              Back
            </Button>
            <div>
              <h1 className="text-xl font-bold">Key Card Details</h1>
              <p className="text-sm text-muted-foreground">ID: {card.id}</p>
            </div>
          </div>
        </div>
      </div>

      <div className="container mx-auto px-4 py-6 space-y-6">
        {/* Guest Info Card */}
        <Card>
          <CardHeader>
            <div className="flex items-center space-x-4">
              <Avatar className="h-16 w-16">
                <AvatarImage src={card.guestAvatar || "/placeholder.svg"} alt={card.guestName} />
                <AvatarFallback className="text-lg">
                  {card.guestName
                    .split(" ")
                    .map((n) => n[0])
                    .join("")}
                </AvatarFallback>
              </Avatar>
              <div className="flex-1">
                <CardTitle className="text-2xl">{card.guestName}</CardTitle>
                <CardDescription className="flex items-center space-x-4 mt-2">
                  <span className="flex items-center">
                    <MapPin className="h-4 w-4 mr-1" />
                    Room {card.roomNumber}
                  </span>
                  <Badge variant="outline" className={`${getStatusColor(card.status)} text-white border-0`}>
                    {getStatusIcon(card.status)}
                    <span className="ml-1 capitalize">{card.status}</span>
                  </Badge>
                </CardDescription>
              </div>
            </div>
          </CardHeader>
        </Card>

        {/* Alerts */}
        {card.batteryLevel < 20 && (
          <Card className="border-red-200 dark:border-red-800">
            <CardContent className="p-4">
              <div className="flex items-center space-x-2 text-red-600 dark:text-red-400">
                <AlertTriangle className="h-5 w-5" />
                <div>
                  <p className="font-semibold">Low Battery Alert</p>
                  <p className="text-sm">Battery level is critically low. Replacement recommended.</p>
                </div>
              </div>
            </CardContent>
          </Card>
        )}

        {isExpiringSoon() && (
          <Card className="border-yellow-200 dark:border-yellow-800">
            <CardContent className="p-4">
              <div className="flex items-center space-x-2 text-yellow-600 dark:text-yellow-400">
                <Clock className="h-5 w-5" />
                <div>
                  <p className="font-semibold">Expiring Soon</p>
                  <p className="text-sm">This key card will expire within 24 hours.</p>
                </div>
              </div>
            </CardContent>
          </Card>
        )}

        {/* Quick Stats */}
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
          <Card>
            <CardContent className="p-4">
              <div className="flex items-center space-x-2">
                {getBatteryIcon(card.batteryLevel)}
                <div>
                  <p className="text-2xl font-bold">{card.batteryLevel}%</p>
                  <p className="text-xs text-muted-foreground">Battery</p>
                </div>
              </div>
              <Progress value={card.batteryLevel} className="mt-2" />
            </CardContent>
          </Card>

          <Card>
            <CardContent className="p-4">
              <div className="flex items-center space-x-2">
                <Activity className="h-4 w-4 text-blue-500" />
                <div>
                  <p className="text-2xl font-bold">{card.accessCount}</p>
                  <p className="text-xs text-muted-foreground">Total Uses</p>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardContent className="p-4">
              <div className="flex items-center space-x-2">
                <Clock className="h-4 w-4 text-green-500" />
                <div>
                  <p className="text-sm font-bold">{formatTime(card.lastAccess)}</p>
                  <p className="text-xs text-muted-foreground">Last Access</p>
                </div>
              </div>
            </CardContent>
          </Card>

          <Card>
            <CardContent className="p-4">
              <div className="flex items-center space-x-2">
                {card.status === "active" ? (
                  <Wifi className="h-4 w-4 text-green-500" />
                ) : (
                  <WifiOff className="h-4 w-4 text-red-500" />
                )}
                <div>
                  <p className="text-sm font-bold capitalize">{card.status}</p>
                  <p className="text-xs text-muted-foreground">Status</p>
                </div>
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Detailed Information */}
        <Tabs defaultValue="details" className="w-full">
          <TabsList className="grid w-full grid-cols-3">
            <TabsTrigger value="details">Details</TabsTrigger>
            <TabsTrigger value="history">Access History</TabsTrigger>
            <TabsTrigger value="actions">Actions</TabsTrigger>
          </TabsList>

          <TabsContent value="details" className="space-y-4">
            <Card>
              <CardHeader>
                <CardTitle>Card Information</CardTitle>
              </CardHeader>
              <CardContent className="space-y-4">
                <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                  <div>
                    <label className="text-sm font-medium text-muted-foreground">Card ID</label>
                    <p className="text-lg font-mono">{card.id}</p>
                  </div>
                  <div>
                    <label className="text-sm font-medium text-muted-foreground">Room Assignment</label>
                    <p className="text-lg">Room {card.roomNumber}</p>
                  </div>
                  <div>
                    <label className="text-sm font-medium text-muted-foreground">Issued Date</label>
                    <p className="text-lg">{formatDate(card.issuedDate)}</p>
                  </div>
                  <div>
                    <label className="text-sm font-medium text-muted-foreground">Expiry Date</label>
                    <p className="text-lg">{formatDate(card.expiryDate)}</p>
                  </div>
                  <div>
                    <label className="text-sm font-medium text-muted-foreground">Guest Name</label>
                    <p className="text-lg">{card.guestName}</p>
                  </div>
                  <div>
                    <label className="text-sm font-medium text-muted-foreground">Current Status</label>
                    <p className="text-lg capitalize">{card.status}</p>
                  </div>
                </div>
              </CardContent>
            </Card>
          </TabsContent>

          <TabsContent value="history" className="space-y-4">
            <Card>
              <CardHeader>
                <CardTitle>Recent Access History</CardTitle>
                <CardDescription>Last {card.accessHistory.length} access events</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="space-y-4">
                  {card.accessHistory.map((access, index) => (
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
                        <p className="text-xs text-muted-foreground">{new Date(access.time).toLocaleDateString()}</p>
                      </div>
                    </div>
                  ))}
                </div>
              </CardContent>
            </Card>
          </TabsContent>

          <TabsContent value="actions" className="space-y-4">
            <Card>
              <CardHeader>
                <CardTitle>Card Actions</CardTitle>
                <CardDescription>Manage this key card</CardDescription>
              </CardHeader>
              <CardContent className="space-y-4">
                <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                  <Button className="w-full bg-transparent" variant="outline">
                    <RefreshCw className="h-4 w-4 mr-2" />
                    Refresh Status
                  </Button>
                  <Button className="w-full bg-transparent" variant="outline">
                    <Edit className="h-4 w-4 mr-2" />
                    Edit Details
                  </Button>
                  <Button className="w-full bg-transparent" variant="outline">
                    <Calendar className="h-4 w-4 mr-2" />
                    Extend Expiry
                  </Button>
                  <Button className="w-full" variant="destructive">
                    <Trash2 className="h-4 w-4 mr-2" />
                    Deactivate Card
                  </Button>
                </div>

                <Separator />

                <div className="space-y-2">
                  <h4 className="font-medium">Emergency Actions</h4>
                  <Button className="w-full" variant="destructive" size="sm">
                    <AlertTriangle className="h-4 w-4 mr-2" />
                    Report Lost/Stolen
                  </Button>
                </div>
              </CardContent>
            </Card>
          </TabsContent>
        </Tabs>
      </div>
    </div>
  )
}
