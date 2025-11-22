"use client"

/**
 * @file realtime-monitoring.tsx
 * @brief Real-time monitoring dashboard for a hotel, showing live
 *        occupancy, power usage, system status, and recent alerts.
 */

import { useState, useEffect, useMemo } from "react"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Progress } from "@/components/ui/progress"
import { Alert, AlertDescription } from "@/components/ui/alert"
import { useHotelData } from "@/hooks/useHotelData"
import { socketService } from "@/lib/socket"
import {
  ChartContainer,
  ChartTooltip,
  ChartTooltipContent,
} from "@/components/ui/chart"
import {
  LineChart,
  Line,
  BarChart,
  Bar,
  XAxis,
  YAxis,
  CartesianGrid,
  ResponsiveContainer,
} from "recharts"
import {
  Activity,
  AlertTriangle,
  CheckCircle,
  Clock,
  Wifi,
  Shield,
  Battery,
  Thermometer,
  Users,
  CreditCard,
  Building2,
  Bell,
  Eye,
  Zap,
} from "lucide-react"

// Mock real-time data
/** Build chart data combining room occupancy state and current usage. */
const buildOccupancyEnergyData = (rooms: any[], powerReadings: any[]) => {
  if (!rooms || rooms.length === 0) return []

  return rooms.map((room) => {
    const power = powerReadings?.find((reading: any) => reading.room === room.number)
    const isActive = room.status === "occupied" || room.status === "maintenance" || room.status === "cleaning"

    return {
      label: room.number,
      occupancy: isActive ? 100 : 0,
      energy: power?.current ?? 0,
    }
  })
}

/** Build multi-series chart data for power chart (temp/humidity style). */
const buildPowerChartData = (rooms: any[], powerReadings: any[]) => {
  if (!rooms || rooms.length === 0) return []

  return rooms.map((room) => {
    const power = powerReadings?.find((reading: any) => reading.room === room.number)

    return {
      label: room.number,
      temperature: power?.current ?? 0,
      humidity: room.hasLowPower ? 1 : 0,
    }
  })
}

/**
 * Build a small list of system alerts derived from recent activities,
 * limited to the most recent entries.
 */
const buildSystemAlerts = (activities: any[]) => {
  if (!activities || activities.length === 0) return []

  return activities.slice(0, 10).map((activity: any) => ({
    id: activity.id,
    type: activity.type === "security" ? "error" : "info",
    title: activity.action,
    description: `User: ${activity.user}`,
    time: activity.time,
    icon: activity.type === "security" ? Shield : Activity,
  }))
}

/** Summarize subsystem health (RFID, power, backend API). */
const buildSystemStatus = (rooms: any[], powerReadings: any[], error: string | null) => {
  const hasRooms = rooms && rooms.length > 0
  const hasPower = powerReadings && powerReadings.length > 0

  const statuses: any[] = []

  statuses.push({
    name: "RFID System",
    status: hasRooms ? "online" : "warning",
    uptime: hasRooms ? "99.9%" : "N/A",
    icon: CreditCard,
  })

  statuses.push({
    name: "Power Monitoring",
    status: hasPower ? "online" : "warning",
    uptime: hasPower ? "99.9%" : "N/A",
    icon: Battery,
  })

  statuses.push({
    name: "Backend API",
    status: error ? "error" : "online",
    uptime: error ? "N/A" : "99.9%",
    icon: Wifi,
  })

  return statuses
}

/** Build live metric cards for occupancy, power, and room counts. */
const buildLiveMetrics = (hotel: any, rooms: any[], powerReadings: any[]) => {
  const totalRooms = hotel?.totalRooms ?? rooms.length
  const occupied = rooms.filter((room: any) => room.status === "occupied").length
  const vacant = rooms.filter((room: any) => room.status === "vacant").length
  const maintenance = rooms.filter((room: any) => room.status === "maintenance" || room.status === "cleaning").length
  const occupancyRate = totalRooms ? Math.round(((occupied + maintenance) / totalRooms) * 100) : 0
  const avgCurrent = powerReadings.length
    ? powerReadings.reduce((sum: number, r: any) => sum + (r.current || 0), 0) / powerReadings.length
    : 0
  const lowPowerRooms = rooms.filter((room: any) => room.hasLowPower).length

  return [
    {
      name: "Current Occupancy",
      value: `${occupancyRate}%`,
      trend: "stable",
      change: `${occupancyRate}%`,
    },
    {
      name: "Rooms with Power On",
      value: `${rooms.filter((room: any) => room.powerStatus === "on").length}`,
      trend: "stable",
      change: "0",
    },
    {
      name: "Low Power Rooms",
      value: `${lowPowerRooms}`,
      trend: "stable",
      change: "0",
    },
    {
      name: "Average Current",
      value: `${avgCurrent.toFixed(2)} A`,
      trend: "stable",
      change: "0.00",
    },
    {
      name: "Total Rooms",
      value: `${totalRooms}`,
      trend: "stable",
      change: "0",
    },
    {
      name: "Vacant Rooms",
      value: `${vacant}`,
      trend: "stable",
      change: "0",
    },
  ]
}

/** Props for the RealtimeMonitoring dashboard. */
interface RealtimeMonitoringProps {
  hotelId: string
}

/**
 * Real-time monitoring view for a hotel using live room, activity, and
 * power readings, plus a heartbeat check on the socket connection.
 */
export function RealtimeMonitoring({ hotelId }: RealtimeMonitoringProps) {
  const { hotel, rooms, activities, loading, error, powerReadings } = useHotelData(hotelId)
  const [currentTime, setCurrentTime] = useState(new Date())
  const [isLive, setIsLive] = useState(true)
  const [isConnected, setIsConnected] = useState<boolean | null>(null)

  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentTime(new Date())
    }, 1000)

    return () => clearInterval(timer)
  }, [])

  useEffect(() => {
    const updateStatus = () => {
      try {
        setIsConnected(socketService.isSocketConnected())
      } catch {
        setIsConnected(null)
      }
    }

    updateStatus()
    const interval = setInterval(updateStatus, 5000)
    return () => clearInterval(interval)
  }, [])

  const occupancyEnergyData = useMemo(
    () => buildOccupancyEnergyData(rooms || [], powerReadings || []),
    [rooms, powerReadings]
  )

  const powerChartData = useMemo(
    () => buildPowerChartData(rooms || [], powerReadings || []),
    [rooms, powerReadings]
  )

  const statusList = useMemo(
    () => buildSystemStatus(rooms || [], powerReadings || [], error),
    [rooms, powerReadings, error]
  )

  const alerts = useMemo(
    () => buildSystemAlerts(activities || []),
    [activities]
  )

  const metrics = useMemo(
    () => buildLiveMetrics(hotel, rooms || [], powerReadings || []),
    [hotel, rooms, powerReadings]
  )

  const getStatusColor = (status: string) => {
    switch (status) {
      case "online":
        return "text-green-600 bg-green-100 dark:bg-green-900/20"
      case "warning":
        return "text-yellow-600 bg-yellow-100 dark:bg-yellow-900/20"
      case "error":
        return "text-red-600 bg-red-100 dark:bg-red-900/20"
      default:
        return "text-gray-600 bg-gray-100 dark:bg-gray-900/20"
    }
  }

  const getAlertVariant = (type: string) => {
    switch (type) {
      case "error":
        return "destructive"
      case "warning":
        return "default"
      case "success":
        return "default"
      case "info":
        return "secondary"
      default:
        return "default"
    }
  }

  const getTrendIcon = (trend: string) => {
    switch (trend) {
      case "up":
        return <Activity className="h-4 w-4 text-green-600" />
      case "down":
        return <Activity className="h-4 w-4 text-red-600" />
      case "stable":
        return <Activity className="h-4 w-4 text-blue-600" />
      default:
        return <Activity className="h-4 w-4 text-gray-600" />
    }
  }

  const chartConfig = {
    occupancy: {
      label: "Occupancy Rate",
      color: "#3b82f6",
    },
    energy: {
      label: "Energy Usage",
      color: "#10b981",
    },
    temperature: {
      label: "Current (A)",
      color: "#f59e0b",
    },
    humidity: {
      label: "Low Power Rooms",
      color: "#8b5cf6",
    },
  }

  return (
    <div className="space-y-6">
      {/* Real-time Header */}
      <div className="flex flex-col lg:flex-row items-start lg:items-center justify-between gap-4">
        <div>
          <h2 className="text-2xl font-bold">Real-time Monitoring</h2>
          <p className="text-muted-foreground">Live system status and performance metrics</p>
        </div>
        <div className="flex items-center space-x-4">
          <div className="flex items-center space-x-2">
            <div
              className={`w-2.5 h-2.5 sm:w-3 sm:h-3 rounded-full flex-shrink-0 ${
                isConnected === null
                  ? "bg-yellow-500"
                  : isConnected
                  ? "bg-green-500"
                  : "bg-red-500"
              }`}
            />
            <span className="text-sm font-medium">
              {isConnected === null
                ? "Connecting..."
                : isConnected
                ? "Backend Connected"
                : "Backend Offline"}
            </span>
          </div>
          <div className="text-sm text-muted-foreground">
            {currentTime.toLocaleTimeString()}
          </div>
          <Button
            variant="outline"
            size="sm"
            onClick={() => setIsLive(!isLive)}
          >
            <Eye className="h-4 w-4 mr-2" />
            {isLive ? "Pause" : "Resume"}
          </Button>
        </div>
      </div>

      {/* Live Metrics Grid */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
        {metrics.map((metric) => (
          <Card key={metric.name}>
            <CardContent className="p-4">
              <div className="flex items-center justify-between">
                <div>
                  <p className="text-sm font-medium text-muted-foreground">{metric.name}</p>
                  <p className="text-2xl font-bold">{metric.value}</p>
                  <div className="flex items-center space-x-1 text-xs">
                    {getTrendIcon(metric.trend)}
                    <span className={metric.trend === "up" ? "text-green-600" : metric.trend === "down" ? "text-red-600" : "text-blue-600"}>
                      {metric.change}
                    </span>
                  </div>
                </div>
                <div className="p-2 bg-blue-100 dark:bg-blue-900/20 rounded-lg">
                  <Activity className="h-5 w-5 text-blue-600" />
                </div>
              </div>
            </CardContent>
          </Card>
        ))}
      </div>

      {/* System Status and Alerts */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* System Status */}
        <Card>
          <CardHeader>
            <CardTitle>System Status</CardTitle>
            <CardDescription>Real-time status of hotel systems</CardDescription>
          </CardHeader>
          <CardContent>
            <div className="space-y-4">
              {statusList.map((system) => {
                const IconComponent = system.icon
                return (
                  <div key={system.name} className="flex items-center justify-between p-3 border rounded-lg">
                    <div className="flex items-center space-x-3">
                      <div className={`p-2 rounded-lg ${getStatusColor(system.status)}`}>
                        <IconComponent className="h-4 w-4" />
                      </div>
                      <div>
                        <p className="font-medium">{system.name}</p>
                        <p className="text-sm text-muted-foreground">Uptime: {system.uptime}</p>
                      </div>
                    </div>
                    <Badge variant={system.status === "online" ? "default" : system.status === "warning" ? "outline" : "destructive"}>
                      {system.status}
                    </Badge>
                  </div>
                )
              })}
            </div>
          </CardContent>
        </Card>

        {/* Live Alerts */}
        <Card>
          <CardHeader>
            <CardTitle>Live Alerts</CardTitle>
            <CardDescription>Recent system notifications and events</CardDescription>
          </CardHeader>
          <CardContent>
            <div className="space-y-4">
              {alerts.map((alert) => {
                const IconComponent = alert.icon
                return (
                  <Alert key={alert.id} variant={getAlertVariant(alert.type) as any}>
                    <IconComponent className="h-4 w-4" />
                    <AlertDescription>
                      <div className="flex items-center justify-between">
                        <div>
                          <p className="font-medium">{alert.title}</p>
                          <p className="text-sm text-muted-foreground">{alert.description}</p>
                        </div>
                        <span className="text-xs text-muted-foreground">{alert.time}</span>
                      </div>
                    </AlertDescription>
                  </Alert>
                )
              })}
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Real-time Charts */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Occupancy and Energy Usage */}
        <Card>
          <CardHeader>
            <CardTitle>Occupancy & Energy Usage</CardTitle>
            <CardDescription>Real-time occupancy and energy consumption</CardDescription>
          </CardHeader>
          <CardContent>
            <ChartContainer config={chartConfig}>
              <LineChart data={occupancyEnergyData}>
                <CartesianGrid strokeDasharray="3 3" className="stroke-muted" />
                <XAxis dataKey="label" className="text-xs" />
                <YAxis className="text-xs" />
                <ChartTooltip content={<ChartTooltipContent />} />
                <Line
                  type="monotone"
                  dataKey="occupancy"
                  stroke="hsl(var(--chart-1))"
                  strokeWidth={2}
                  dot={{ fill: "hsl(var(--chart-1))", strokeWidth: 2, r: 3 }}
                />
                <Line
                  type="monotone"
                  dataKey="energy"
                  stroke="hsl(var(--chart-2))"
                  strokeWidth={2}
                  dot={{ fill: "hsl(var(--chart-2))", strokeWidth: 2, r: 3 }}
                />
              </LineChart>
            </ChartContainer>
          </CardContent>
        </Card>

        {/* Environmental Conditions */}
        <Card>
          <CardHeader>
            <CardTitle>Environmental Conditions</CardTitle>
            <CardDescription>Temperature and humidity monitoring</CardDescription>
          </CardHeader>
          <CardContent>
            <ChartContainer config={chartConfig}>
              <BarChart data={powerChartData}>
                <CartesianGrid strokeDasharray="3 3" className="stroke-muted" />
                <XAxis dataKey="label" className="text-xs" />
                <YAxis className="text-xs" />
                <ChartTooltip content={<ChartTooltipContent />} />
                <Bar dataKey="temperature" fill="hsl(var(--chart-3))" name="Current (A)" />
                <Bar dataKey="humidity" fill="hsl(var(--chart-4))" name="Low Power (1 = yes)" />
              </BarChart>
            </ChartContainer>
          </CardContent>
        </Card>
      </div>

      {/* Quick Actions */}
      <Card>
        <CardHeader>
          <CardTitle>Quick Actions</CardTitle>
          <CardDescription>Common monitoring and control actions</CardDescription>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4">
            <Button variant="outline" className="h-auto p-4 flex-col space-y-2">
              <Shield className="h-6 w-6" />
              <span>Security Check</span>
            </Button>
            <Button variant="outline" className="h-auto p-4 flex-col space-y-2">
              <Thermometer className="h-6 w-6" />
              <span>HVAC Control</span>
            </Button>
            <Button variant="outline" className="h-auto p-4 flex-col space-y-2">
              <Zap className="h-6 w-6" />
              <span>Energy Report</span>
            </Button>
            <Button variant="outline" className="h-auto p-4 flex-col space-y-2">
              <Bell className="h-6 w-6" />
              <span>Alert Settings</span>
            </Button>
          </div>
        </CardContent>
      </Card>
    </div>
  )
} 