"use client"

import { useState } from "react"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { Badge } from "@/components/ui/badge"
import { Progress } from "@/components/ui/progress"
import { useHotelData } from "@/hooks/useHotelData"
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
  AreaChart,
  Area,
  XAxis,
  YAxis,
  CartesianGrid,
  ResponsiveContainer,
  ScatterChart,
  Scatter,
  PieChart,
  Pie,
  Cell,
} from "recharts"
import {
  TrendingUp,
  TrendingDown,
  Target,
  AlertTriangle,
  CheckCircle,
  Clock,
  DollarSign,
  Users,
} from "lucide-react"

// Mock data for performance metrics
const performanceData = [
  { month: "Jan", efficiency: 85, satisfaction: 4.2, revenue: 45000, costs: 32000 },
  { month: "Feb", efficiency: 88, satisfaction: 4.4, revenue: 52000, costs: 35000 },
  { month: "Mar", efficiency: 92, satisfaction: 4.6, revenue: 58000, costs: 38000 },
  { month: "Apr", efficiency: 89, satisfaction: 4.5, revenue: 65000, costs: 42000 },
  { month: "May", efficiency: 94, satisfaction: 4.7, revenue: 72000, costs: 45000 },
  { month: "Jun", efficiency: 91, satisfaction: 4.6, revenue: 68000, costs: 43000 },
  { month: "Jul", efficiency: 96, satisfaction: 4.8, revenue: 78000, costs: 48000 },
  { month: "Aug", efficiency: 93, satisfaction: 4.7, revenue: 72000, costs: 46000 },
  { month: "Sep", efficiency: 90, satisfaction: 4.5, revenue: 62000, costs: 40000 },
  { month: "Oct", efficiency: 87, satisfaction: 4.3, revenue: 55000, costs: 37000 },
  { month: "Nov", efficiency: 84, satisfaction: 4.1, revenue: 48000, costs: 34000 },
  { month: "Dec", efficiency: 89, satisfaction: 4.4, revenue: 60000, costs: 41000 },
]

const kpiData = [
  { name: "Occupancy Rate", current: 78, target: 85, trend: "up", color: "#3b82f6" },
  { name: "Guest Satisfaction", current: 4.7, target: 4.8, trend: "up", color: "#10b981" },
  { name: "Revenue per Room", current: 145, target: 160, trend: "up", color: "#f59e0b" },
  { name: "Staff Efficiency", current: 92, target: 90, trend: "up", color: "#8b5cf6" },
  { name: "Energy Efficiency", current: 85, target: 88, trend: "down", color: "#ef4444" },
  { name: "Maintenance Score", current: 88, target: 85, trend: "up", color: "#06b6d4" },
]

const departmentPerformance = [
  { department: "Front Desk", efficiency: 95, satisfaction: 4.8, response: 2.1 },
  { department: "Housekeeping", efficiency: 88, satisfaction: 4.6, response: 3.5 },
  { department: "Food & Beverage", efficiency: 92, satisfaction: 4.7, response: 8.2 },
  { department: "Maintenance", efficiency: 85, satisfaction: 4.4, response: 15.3 },
  { department: "Spa & Wellness", efficiency: 90, satisfaction: 4.9, response: 5.1 },
]

const seasonalTrends = [
  { season: "Spring", occupancy: 75, revenue: 55000, guests: 140 },
  { season: "Summer", occupancy: 92, revenue: 78000, guests: 180 },
  { season: "Fall", occupancy: 68, revenue: 48000, guests: 125 },
  { season: "Winter", occupancy: 82, revenue: 62000, guests: 155 },
]

interface PerformanceMetricsProps {
  hotelId: string
}

export function PerformanceMetrics({ hotelId }: PerformanceMetricsProps) {
  const { hotel, rooms, activities, loading, error } = useHotelData(hotelId)
  const [selectedPeriod, setSelectedPeriod] = useState("12months")
  const [selectedMetric, setSelectedMetric] = useState("efficiency")

  const chartConfig = {
    efficiency: {
      label: "Operational Efficiency",
      color: "#3b82f6",
    },
    satisfaction: {
      label: "Guest Satisfaction",
      color: "#10b981",
    },
    revenue: {
      label: "Revenue",
      color: "#f59e0b",
    },
    costs: {
      label: "Operating Costs",
      color: "#ef4444",
    },
  }

  const getKpiStatus = (current: number, target: number, trend: string) => {
    const percentage = (current / target) * 100
    if (percentage >= 100) return "excellent"
    if (percentage >= 90) return "good"
    if (percentage >= 80) return "warning"
    return "critical"
  }

  const getStatusColor = (status: string) => {
    switch (status) {
      case "excellent":
        return "bg-green-500"
      case "good":
        return "bg-blue-500"
      case "warning":
        return "bg-yellow-500"
      case "critical":
        return "bg-red-500"
      default:
        return "bg-gray-500"
    }
  }

  const getTrendIcon = (trend: string) => {
    return trend === "up" ? (
      <TrendingUp className="h-4 w-4 text-green-600" />
    ) : (
      <TrendingDown className="h-4 w-4 text-red-600" />
    )
  }

  return (
    <div className="space-y-6">
      {/* Performance Header */}
      <div className="flex flex-col lg:flex-row items-start lg:items-center justify-between gap-4">
        <div>
          <h2 className="text-2xl font-bold">Performance Metrics</h2>
          <p className="text-muted-foreground">Key performance indicators and operational efficiency</p>
        </div>
        <Select value={selectedPeriod} onValueChange={setSelectedPeriod}>
          <SelectTrigger className="w-full sm:w-[180px]">
            <SelectValue placeholder="Time Period" />
          </SelectTrigger>
          <SelectContent>
            <SelectItem value="30days">Last 30 Days</SelectItem>
            <SelectItem value="3months">Last 3 Months</SelectItem>
            <SelectItem value="6months">Last 6 Months</SelectItem>
            <SelectItem value="12months">Last 12 Months</SelectItem>
          </SelectContent>
        </Select>
      </div>

      {/* KPI Dashboard */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
        {kpiData.map((kpi) => {
          const status = getKpiStatus(kpi.current, kpi.target, kpi.trend)
          const percentage = (kpi.current / kpi.target) * 100
          
          return (
            <Card key={kpi.name}>
              <CardContent className="p-4">
                <div className="flex items-center justify-between mb-3">
                  <h3 className="font-medium text-sm">{kpi.name}</h3>
                  {getTrendIcon(kpi.trend)}
                </div>
                <div className="space-y-2">
                  <div className="flex items-center justify-between">
                    <span className="text-2xl font-bold">{kpi.current}</span>
                    <span className="text-sm text-muted-foreground">Target: {kpi.target}</span>
                  </div>
                  <Progress value={Math.min(percentage, 100)} className="h-2" />
                  <div className="flex items-center justify-between text-xs">
                    <span className="text-muted-foreground">{percentage.toFixed(1)}% of target</span>
                    <Badge variant={status === "excellent" ? "default" : status === "good" ? "secondary" : status === "warning" ? "outline" : "destructive"}>
                      {status}
                    </Badge>
                  </div>
                </div>
              </CardContent>
            </Card>
          )
        })}
      </div>

      {/* Performance Charts */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Trend Analysis */}
        <Card>
          <CardHeader>
            <div className="flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4">
              <div>
                <CardTitle>Performance Trends</CardTitle>
                <CardDescription>Monthly performance metrics over time</CardDescription>
              </div>
              <Select value={selectedMetric} onValueChange={setSelectedMetric}>
                <SelectTrigger className="w-full sm:w-[180px]">
                  <SelectValue placeholder="Select Metric" />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="efficiency">Efficiency</SelectItem>
                  <SelectItem value="satisfaction">Satisfaction</SelectItem>
                  <SelectItem value="revenue">Revenue</SelectItem>
                  <SelectItem value="costs">Costs</SelectItem>
                </SelectContent>
              </Select>
            </div>
          </CardHeader>
          <CardContent>
            <ChartContainer config={chartConfig}>
              <LineChart data={performanceData}>
                <CartesianGrid strokeDasharray="3 3" className="stroke-muted" />
                <XAxis dataKey="month" className="text-xs" />
                <YAxis className="text-xs" />
                <ChartTooltip content={<ChartTooltipContent />} />
                <Line
                  type="monotone"
                  dataKey={selectedMetric}
                  stroke="hsl(var(--chart-1))"
                  strokeWidth={2}
                  dot={{ fill: "hsl(var(--chart-1))", strokeWidth: 2, r: 4 }}
                />
              </LineChart>
            </ChartContainer>
          </CardContent>
        </Card>

        {/* Department Performance */}
        <Card>
          <CardHeader>
            <CardTitle>Department Performance</CardTitle>
            <CardDescription>Efficiency and satisfaction by department</CardDescription>
          </CardHeader>
          <CardContent>
            <ChartContainer config={chartConfig}>
              <BarChart data={departmentPerformance}>
                <CartesianGrid strokeDasharray="3 3" className="stroke-muted" />
                <XAxis dataKey="department" className="text-xs" />
                <YAxis className="text-xs" />
                <ChartTooltip content={<ChartTooltipContent />} />
                <Bar dataKey="efficiency" fill="#3b82f6" name="Efficiency %" />
                <Bar dataKey="satisfaction" fill="#10b981" name="Satisfaction" />
              </BarChart>
            </ChartContainer>
          </CardContent>
        </Card>

        {/* Seasonal Analysis */}
        <Card>
          <CardHeader>
            <CardTitle>Seasonal Performance</CardTitle>
            <CardDescription>Performance metrics by season</CardDescription>
          </CardHeader>
          <CardContent>
            <ChartContainer config={chartConfig}>
              <AreaChart data={seasonalTrends}>
                <CartesianGrid strokeDasharray="3 3" className="stroke-muted" />
                <XAxis dataKey="season" className="text-xs" />
                <YAxis className="text-xs" />
                <ChartTooltip content={<ChartTooltipContent />} />
                <Area
                  type="monotone"
                  dataKey="occupancy"
                  fill="hsl(var(--chart-1))"
                  fillOpacity={0.3}
                  stroke="hsl(var(--chart-1))"
                  strokeWidth={2}
                />
                <Area
                  type="monotone"
                  dataKey="guests"
                  fill="hsl(var(--chart-2))"
                  fillOpacity={0.3}
                  stroke="hsl(var(--chart-2))"
                  strokeWidth={2}
                />
              </AreaChart>
            </ChartContainer>
          </CardContent>
        </Card>

        {/* Efficiency vs Satisfaction Scatter */}
        <Card>
          <CardHeader>
            <CardTitle>Efficiency vs Satisfaction</CardTitle>
            <CardDescription>Correlation between operational efficiency and guest satisfaction</CardDescription>
          </CardHeader>
          <CardContent>
            <ChartContainer config={chartConfig}>
              <ScatterChart data={performanceData}>
                <CartesianGrid strokeDasharray="3 3" className="stroke-muted" />
                <XAxis dataKey="efficiency" name="Efficiency" className="text-xs" />
                <YAxis dataKey="satisfaction" name="Satisfaction" className="text-xs" />
                <ChartTooltip content={<ChartTooltipContent />} />
                <Scatter dataKey="satisfaction" fill="hsl(var(--chart-1))" />
              </ScatterChart>
            </ChartContainer>
          </CardContent>
        </Card>
      </div>

      {/* Department Details */}
      <Card>
        <CardHeader>
          <CardTitle>Department Performance Details</CardTitle>
          <CardDescription>Detailed metrics for each department</CardDescription>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
            {departmentPerformance.map((dept) => (
              <div key={dept.department} className="p-4 border rounded-lg">
                <div className="flex items-center justify-between mb-3">
                  <h4 className="font-medium">{dept.department}</h4>
                  <Badge variant={dept.efficiency >= 90 ? "default" : dept.efficiency >= 80 ? "secondary" : "outline"}>
                    {dept.efficiency}%
                  </Badge>
                </div>
                <div className="space-y-2">
                  <div className="flex justify-between text-sm">
                    <span className="text-muted-foreground">Efficiency:</span>
                    <span className="font-medium">{dept.efficiency}%</span>
                  </div>
                  <div className="flex justify-between text-sm">
                    <span className="text-muted-foreground">Satisfaction:</span>
                    <span className="font-medium">{dept.satisfaction}/5.0</span>
                  </div>
                  <div className="flex justify-between text-sm">
                    <span className="text-muted-foreground">Avg Response:</span>
                    <span className="font-medium">{dept.response} min</span>
                  </div>
                </div>
              </div>
            ))}
          </div>
        </CardContent>
      </Card>
    </div>
  )
} 