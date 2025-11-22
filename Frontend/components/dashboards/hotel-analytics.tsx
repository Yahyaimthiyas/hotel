"use client"

import { useState } from "react"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { Button } from "@/components/ui/button"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import { Progress } from "@/components/ui/progress"
import { PerformanceMetrics } from "./performance-metrics"
import { RealtimeMonitoring } from "./realtime-monitoring"
import { useHotelData } from "@/hooks/useHotelData"
import {
  ChartContainer,
  ChartTooltip,
  ChartTooltipContent,
} from "@/components/ui/chart"
import {
  BarChart,
  Bar,
  LineChart,
  Line,
  AreaChart,
  Area,
  PieChart,
  Pie,
  Cell,
  XAxis,
  YAxis,
  CartesianGrid,
  ResponsiveContainer,
  RadarChart,
  PolarGrid,
  PolarAngleAxis,
  PolarRadiusAxis,
  Radar,
  ComposedChart,
} from "recharts"
import {
  TrendingUp,
  Users,
  DollarSign,
  Star,
  CreditCard,
  Download,
} from "lucide-react"

// Mock data for analytics
const occupancyData = [
  { month: "Jan", occupancy: 65, revenue: 45000, guests: 120 },
  { month: "Feb", occupancy: 72, revenue: 52000, guests: 135 },
  { month: "Mar", occupancy: 78, revenue: 58000, guests: 145 },
  { month: "Apr", occupancy: 85, revenue: 65000, guests: 160 },
  { month: "May", occupancy: 92, revenue: 72000, guests: 175 },
  { month: "Jun", occupancy: 88, revenue: 68000, guests: 165 },
  { month: "Jul", occupancy: 95, revenue: 78000, guests: 180 },
  { month: "Aug", occupancy: 90, revenue: 72000, guests: 170 },
  { month: "Sep", occupancy: 82, revenue: 62000, guests: 155 },
  { month: "Oct", occupancy: 75, revenue: 55000, guests: 140 },
  { month: "Nov", occupancy: 68, revenue: 48000, guests: 125 },
  { month: "Dec", occupancy: 80, revenue: 60000, guests: 150 },
]

const revenueData = [
  { month: "Jan", revenue: 45000 },
  { month: "Feb", revenue: 52000 },
  { month: "Mar", revenue: 58000 },
  { month: "Apr", revenue: 65000 },
  { month: "May", revenue: 72000 },
  { month: "Jun", revenue: 68000 },
  { month: "Jul", revenue: 78000 },
  { month: "Aug", revenue: 72000 },
  { month: "Sep", revenue: 62000 },
  { month: "Oct", revenue: 55000 },
  { month: "Nov", revenue: 48000 },
  { month: "Dec", revenue: 60000 },
]

const activityData = [
  { hour: "00:00", checkins: 2, checkouts: 1, services: 3 },
  { hour: "04:00", checkins: 0, checkouts: 0, services: 1 },
  { hour: "08:00", checkins: 8, checkouts: 12, services: 15 },
  { hour: "12:00", checkins: 15, checkouts: 8, services: 25 },
  { hour: "16:00", checkins: 20, checkouts: 5, services: 18 },
  { hour: "20:00", checkins: 5, checkouts: 2, services: 12 },
]

const roomTypeData = [
  { name: "Standard", value: 45, color: "#3b82f6" },
  { name: "Deluxe", value: 30, color: "#10b981" },
  { name: "Suite", value: 15, color: "#f59e0b" },
  { name: "Presidential", value: 10, color: "#ef4444" },
]

const chartColors = [
  "#3b82f6", // blue
  "#10b981", // green
  "#f59e0b", // yellow
  "#ef4444", // red
  "#8b5cf6", // purple
  "#06b6d4", // cyan
  "#f97316", // orange
  "#ec4899", // pink
]

const serviceRatings = [
  { service: "Cleanliness", rating: 4.8, target: 5.0 },
  { service: "Staff Service", rating: 4.6, target: 5.0 },
  { service: "Room Comfort", rating: 4.7, target: 5.0 },
  { service: "Food Quality", rating: 4.5, target: 5.0 },
  { service: "Location", rating: 4.9, target: 5.0 },
  { service: "Value for Money", rating: 4.4, target: 5.0 },
]

const dailyActivityData = [
  { hour: "00:00", checkins: 2, checkouts: 1, services: 3 },
  { hour: "04:00", checkins: 0, checkouts: 0, services: 1 },
  { hour: "08:00", checkins: 8, checkouts: 12, services: 15 },
  { hour: "12:00", checkins: 15, checkouts: 8, services: 25 },
  { hour: "16:00", checkins: 20, checkouts: 5, services: 18 },
  { hour: "20:00", checkins: 5, checkouts: 2, services: 12 },
]

interface HotelAnalyticsProps {
  hotelId: string
}

export function HotelAnalytics({ hotelId }: HotelAnalyticsProps) {
  const { hotel, rooms, activities, loading, error } = useHotelData(hotelId)
  const [timeRange, setTimeRange] = useState("12months")
  const [selectedMetric, setSelectedMetric] = useState("occupancy")
  const [activeTab, setActiveTab] = useState("overview")

  const chartConfig = {
    occupancy: {
      label: "Occupancy Rate",
      color: "#3b82f6",
    },
    revenue: {
      label: "Revenue",
      color: "#10b981",
    },
    guests: {
      label: "Guests",
      color: "#f59e0b",
    },
  }

  return (
    <div className="h-full flex flex-col">
      {/* Analytics Header - Compact and Fixed */}
      <div className="flex-shrink-0 mb-4">
        <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-3">
          <div>
            <h2 className="text-lg sm:text-xl md:text-2xl font-bold leading-tight">Hotel Analytics</h2>
            <p className="text-xs sm:text-sm md:text-base text-muted-foreground mt-1">Comprehensive insights and performance metrics</p>
          </div>
          <div className="flex flex-col sm:flex-row gap-2">
            <Select value={timeRange} onValueChange={setTimeRange}>
              <SelectTrigger className="w-full sm:w-[140px] md:w-[160px] lg:w-[180px] text-xs sm:text-sm h-8 sm:h-10">
                <SelectValue placeholder="Time Range" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="7days">Last 7 Days</SelectItem>
                <SelectItem value="30days">Last 30 Days</SelectItem>
                <SelectItem value="3months">Last 3 Months</SelectItem>
                <SelectItem value="6months">Last 6 Months</SelectItem>
                <SelectItem value="12months">Last 12 Months</SelectItem>
              </SelectContent>
            </Select>
            <Button variant="outline" size="sm" className="w-full sm:w-auto text-xs sm:text-sm h-8 sm:h-10">
              <Download className="h-3 w-3 sm:h-4 sm:w-4 mr-1 sm:mr-2" />
              <span className="hidden sm:inline">Export Data</span>
              <span className="sm:hidden">Export</span>
            </Button>
          </div>
        </div>
      </div>

      {/* Key Metrics Cards - Compact Layout */}
      <div className="flex-shrink-0 mb-4">
        <div className="grid grid-cols-2 lg:grid-cols-4 gap-2 sm:gap-3 md:gap-4">
        <Card className="min-h-[70px] sm:min-h-[80px]">
          <CardContent className="p-2 sm:p-3 h-full">
            <div className="flex items-center justify-between h-full">
              <div className="min-w-0 flex-1">
                <p className="text-xs sm:text-sm font-medium text-muted-foreground truncate leading-tight">Average Occupancy</p>
                <p className="text-base sm:text-lg md:text-2xl font-bold truncate leading-tight mt-1">78.5%</p>
                <div className="flex items-center text-xs sm:text-sm text-green-600 mt-1">
                  <TrendingUp className="h-3 w-3 sm:h-4 sm:w-4 mr-1 flex-shrink-0" />
                  <span className="truncate">+5.2%</span>
                </div>
              </div>
              <div className="p-1 sm:p-1.5 md:p-2 bg-blue-100 dark:bg-blue-900/20 rounded-lg flex-shrink-0 ml-2">
                <Users className="h-3 w-3 sm:h-4 sm:w-4 md:h-5 md:w-5 text-blue-600" />
              </div>
            </div>
          </CardContent>
        </Card>

        <Card className="min-h-[70px] sm:min-h-[80px]">
          <CardContent className="p-2 sm:p-3 h-full">
            <div className="flex items-center justify-between h-full">
              <div className="min-w-0 flex-1">
                <p className="text-xs sm:text-sm font-medium text-muted-foreground truncate leading-tight">Monthly Revenue</p>
                <p className="text-base sm:text-lg md:text-2xl font-bold truncate leading-tight mt-1">$72,450</p>
                <div className="flex items-center text-xs sm:text-sm text-green-600 mt-1">
                  <TrendingUp className="h-3 w-3 sm:h-4 sm:w-4 mr-1 flex-shrink-0" />
                  <span className="truncate">+12.3%</span>
                </div>
              </div>
              <div className="p-1 sm:p-1.5 md:p-2 bg-green-100 dark:bg-green-900/20 rounded-lg flex-shrink-0 ml-2">
                <DollarSign className="h-3 w-3 sm:h-4 sm:w-4 md:h-5 md:w-5 text-green-600" />
              </div>
            </div>
          </CardContent>
        </Card>

        <Card className="min-h-[70px] sm:min-h-[80px]">
          <CardContent className="p-2 sm:p-3 h-full">
            <div className="flex items-center justify-between h-full">
              <div className="min-w-0 flex-1">
                <p className="text-xs sm:text-sm font-medium text-muted-foreground truncate leading-tight">Guest Satisfaction</p>
                <p className="text-base sm:text-lg md:text-2xl font-bold truncate leading-tight mt-1">4.7/5.0</p>
                <div className="flex items-center text-xs sm:text-sm text-green-600 mt-1">
                  <TrendingUp className="h-3 w-3 sm:h-4 sm:w-4 mr-1 flex-shrink-0" />
                  <span className="truncate">+0.2</span>
                </div>
              </div>
              <div className="p-1 sm:p-1.5 md:p-2 bg-yellow-100 dark:bg-yellow-900/20 rounded-lg flex-shrink-0 ml-2">
                <Star className="h-3 w-3 sm:h-4 sm:w-4 md:h-5 md:w-5 text-yellow-600" />
              </div>
            </div>
          </CardContent>
        </Card>

        <Card className="min-h-[70px] sm:min-h-[80px]">
          <CardContent className="p-2 sm:p-3 h-full">
            <div className="flex items-center justify-between h-full">
              <div className="min-w-0 flex-1">
                <p className="text-xs sm:text-sm font-medium text-muted-foreground truncate leading-tight">Active Cards</p>
                <p className="text-base sm:text-lg md:text-2xl font-bold truncate leading-tight mt-1">156</p>
                <div className="flex items-center text-xs sm:text-sm text-green-600 mt-1">
                  <TrendingUp className="h-3 w-3 sm:h-4 sm:w-4 mr-1 flex-shrink-0" />
                  <span className="truncate">+8.5%</span>
                </div>
              </div>
              <div className="p-1 sm:p-1.5 md:p-2 bg-purple-100 dark:bg-purple-900/20 rounded-lg flex-shrink-0 ml-2">
                <CreditCard className="h-3 w-3 sm:h-4 sm:w-4 md:h-5 md:w-5 text-purple-600" />
              </div>
            </div>
          </CardContent>
        </Card>
        </div>
      </div>

      {/* Analytics Tabs - Scrollable Content Area */}
      <div className="flex-1 min-h-0">
        <Tabs value={activeTab} onValueChange={setActiveTab} className="h-full flex flex-col">
          <div className="flex-shrink-0 mb-4">
            {/* Desktop: Full Tab List */}
            <div className="hidden lg:block">
              <TabsList className="flex w-full justify-start gap-2 p-1.5 bg-muted/50 border border-border/50 rounded-lg">
                <TabsTrigger value="overview" className="flex-1 text-sm font-medium px-4 py-2.5 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground">
                  📊 Overview
                </TabsTrigger>
                <TabsTrigger value="performance" className="flex-1 text-sm font-medium px-4 py-2.5 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground">
                  📈 Performance
                </TabsTrigger>
                <TabsTrigger value="realtime" className="flex-1 text-sm font-medium px-4 py-2.5 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground">
                  ⚡ Live
                </TabsTrigger>
                <TabsTrigger value="occupancy" className="flex-1 text-sm font-medium px-4 py-2.5 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground">
                  🏠 Occupancy
                </TabsTrigger>
                <TabsTrigger value="revenue" className="flex-1 text-sm font-medium px-4 py-2.5 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground">
                  💰 Revenue
                </TabsTrigger>
                <TabsTrigger value="guests" className="flex-1 text-sm font-medium px-4 py-2.5 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground">
                  👥 Guests
                </TabsTrigger>
                <TabsTrigger value="services" className="flex-1 text-sm font-medium px-4 py-2.5 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground">
                  🛎️ Services
                </TabsTrigger>
                <TabsTrigger value="activity" className="flex-1 text-sm font-medium px-4 py-2.5 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground">
                  📋 Activity
                </TabsTrigger>
              </TabsList>
            </div>

            {/* Tablet: Compact Tab List */}
            <div className="hidden sm:block lg:hidden">
              <TabsList className="flex w-full justify-start gap-1.5 p-1 bg-muted/50 border border-border/50 rounded-lg overflow-x-auto">
                <TabsTrigger value="overview" className="text-xs font-medium px-3 py-2 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground whitespace-nowrap">
                  📊 Overview
                </TabsTrigger>
                <TabsTrigger value="performance" className="text-xs font-medium px-3 py-2 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground whitespace-nowrap">
                  📈 Performance
                </TabsTrigger>
                <TabsTrigger value="realtime" className="text-xs font-medium px-3 py-2 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground whitespace-nowrap">
                  ⚡ Live
                </TabsTrigger>
                <TabsTrigger value="occupancy" className="text-xs font-medium px-3 py-2 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground whitespace-nowrap">
                  🏠 Occupancy
                </TabsTrigger>
                <TabsTrigger value="revenue" className="text-xs font-medium px-3 py-2 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground whitespace-nowrap">
                  💰 Revenue
                </TabsTrigger>
                <TabsTrigger value="guests" className="text-xs font-medium px-3 py-2 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground whitespace-nowrap">
                  👥 Guests
                </TabsTrigger>
                <TabsTrigger value="services" className="text-xs font-medium px-3 py-2 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground whitespace-nowrap">
                  🛎️ Services
                </TabsTrigger>
                <TabsTrigger value="activity" className="text-xs font-medium px-3 py-2 rounded-md transition-all duration-200 hover:bg-background/80 data-[state=active]:bg-background data-[state=active]:shadow-sm data-[state=active]:text-foreground data-[state=inactive]:text-muted-foreground hover:data-[state=inactive]:text-foreground whitespace-nowrap">
                  📋 Activity
                </TabsTrigger>
              </TabsList>
            </div>

            {/* Mobile: Dropdown Menu */}
            <div className="sm:hidden">
              <div className="flex items-center space-x-2">
                <div className="flex-1">
                  <Select value={activeTab} onValueChange={setActiveTab}>
                    <SelectTrigger className="w-full text-sm h-10 bg-muted/50 border border-border/50 rounded-lg">
                      <SelectValue placeholder="Select analytics section" />
                    </SelectTrigger>
                    <SelectContent className="bg-background border border-border rounded-lg shadow-lg">
                      <SelectItem value="overview" className="text-sm py-2.5">📊 Overview</SelectItem>
                      <SelectItem value="performance" className="text-sm py-2.5">📈 Performance</SelectItem>
                      <SelectItem value="realtime" className="text-sm py-2.5">⚡ Live</SelectItem>
                      <SelectItem value="occupancy" className="text-sm py-2.5">🏠 Occupancy</SelectItem>
                      <SelectItem value="revenue" className="text-sm py-2.5">💰 Revenue</SelectItem>
                      <SelectItem value="guests" className="text-sm py-2.5">👥 Guests</SelectItem>
                      <SelectItem value="services" className="text-sm py-2.5">🛎️ Services</SelectItem>
                      <SelectItem value="activity" className="text-sm py-2.5">📋 Activity</SelectItem>
                    </SelectContent>
                  </Select>
                </div>
              </div>
            </div>
          </div>

                    <div className="flex-1 min-h-0 overflow-y-auto analytics-container">
            <TabsContent value="overview" className="space-y-3 sm:space-y-4 h-full">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-3 sm:gap-4 md:gap-6">
            {/* Occupancy Trend Chart */}
            <Card className="min-h-[250px] sm:min-h-[280px] md:min-h-[320px]">
              <CardHeader className="pb-2 sm:pb-3">
                <CardTitle className="text-sm sm:text-lg leading-tight">Occupancy Trends</CardTitle>
                <CardDescription className="text-xs sm:text-sm leading-tight">Monthly occupancy rates over the past year</CardDescription>
              </CardHeader>
              <CardContent className="h-full">
                <div className="h-[200px] sm:h-[220px] md:h-[250px]">
                  <ChartContainer config={chartConfig}>
                    <LineChart data={occupancyData}>
                      <CartesianGrid strokeDasharray="3 3" className="stroke-muted" />
                      <XAxis dataKey="month" className="text-xs" />
                      <YAxis className="text-xs" />
                      <ChartTooltip content={<ChartTooltipContent />} />
                      <Line
                        type="monotone"
                        dataKey="occupancy"
                        stroke="hsl(var(--chart-1))"
                        strokeWidth={2}
                        dot={{ fill: "hsl(var(--chart-1))", strokeWidth: 2, r: 3 }}
                      />
                    </LineChart>
                  </ChartContainer>
                </div>
              </CardContent>
            </Card>

            {/* Room Type Distribution */}
            <Card className="min-h-[250px] sm:min-h-[280px] md:min-h-[320px]">
              <CardHeader className="pb-2 sm:pb-3">
                <CardTitle className="text-sm sm:text-lg leading-tight">Room Type Distribution</CardTitle>
                <CardDescription className="text-xs sm:text-sm leading-tight">Current room type allocation</CardDescription>
              </CardHeader>
              <CardContent className="h-full">
                <div className="h-[200px] sm:h-[220px] md:h-[250px]">
                  <ChartContainer config={chartConfig}>
                    <PieChart data={roomTypeData}>
                      <Pie
                        data={roomTypeData}
                        cx="50%"
                        cy="50%"
                        outerRadius={60}
                        fill="#8884d8"
                        dataKey="value"
                        label={({ name, percent }) => `${name} ${(percent * 100).toFixed(0)}%`}
                      >
                        {roomTypeData.map((entry, index) => (
                          <Cell key={`cell-${index}`} fill={chartColors[index % chartColors.length]} />
                        ))}
                      </Pie>
                      <ChartTooltip content={<ChartTooltipContent />} />
                    </PieChart>
                  </ChartContainer>
                </div>
              </CardContent>
            </Card>
          </div>
        </TabsContent>

            <TabsContent value="performance" className="space-y-3 sm:space-y-4 h-full">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-3 sm:gap-4 md:gap-6">
            {/* Performance Overview */}
            <Card className="min-h-[250px] sm:min-h-[280px] md:min-h-[320px]">
              <CardHeader className="pb-2 sm:pb-3">
                <CardTitle className="text-sm sm:text-lg leading-tight">Performance Overview</CardTitle>
                <CardDescription className="text-xs sm:text-sm leading-tight">Key performance indicators over time</CardDescription>
              </CardHeader>
              <CardContent className="h-full">
                <div className="h-[200px] sm:h-[220px] md:h-[250px]">
                  <ChartContainer config={chartConfig}>
                    <ComposedChart data={occupancyData}>
                      <CartesianGrid strokeDasharray="3 3" className="stroke-muted" />
                      <XAxis dataKey="month" className="text-xs" />
                      <YAxis className="text-xs" />
                      <ChartTooltip content={<ChartTooltipContent />} />
                      <Line
                        type="monotone"
                        dataKey="occupancy"
                        stroke="hsl(var(--chart-1))"
                        strokeWidth={2}
                        name="Occupancy Rate"
                      />
                      <Line
                        type="monotone"
                        dataKey="guests"
                        stroke="hsl(var(--chart-2))"
                        strokeWidth={2}
                        name="Guest Count"
                      />
                    </ComposedChart>
                  </ChartContainer>
                </div>
              </CardContent>
            </Card>

            {/* Performance Metrics */}
            <Card className="min-h-[250px] sm:min-h-[280px] md:min-h-[320px]">
              <CardHeader className="pb-2 sm:pb-3">
                <CardTitle className="text-sm sm:text-lg leading-tight">Performance Metrics</CardTitle>
                <CardDescription className="text-xs sm:text-sm leading-tight">Detailed performance breakdown</CardDescription>
              </CardHeader>
              <CardContent className="space-y-3 sm:space-y-4">
                <div className="grid grid-cols-2 gap-2 sm:gap-3">
                  <div className="p-2 sm:p-3 bg-blue-50 dark:bg-blue-900/20 rounded-lg">
                    <p className="text-xs sm:text-sm text-muted-foreground">Avg Daily Revenue</p>
                    <p className="text-lg sm:text-xl font-bold text-blue-600">$2,415</p>
                    <p className="text-xs text-green-600">+8.2% vs last month</p>
                  </div>
                  <div className="p-2 sm:p-3 bg-green-50 dark:bg-green-900/20 rounded-lg">
                    <p className="text-xs sm:text-sm text-muted-foreground">RevPAR</p>
                    <p className="text-lg sm:text-xl font-bold text-green-600">$189.50</p>
                    <p className="text-xs text-green-600">+12.1% vs last month</p>
                  </div>
                  <div className="p-2 sm:p-3 bg-purple-50 dark:bg-purple-900/20 rounded-lg">
                    <p className="text-xs sm:text-sm text-muted-foreground">ADR</p>
                    <p className="text-lg sm:text-xl font-bold text-purple-600">$243.20</p>
                    <p className="text-xs text-green-600">+5.8% vs last month</p>
                  </div>
                  <div className="p-2 sm:p-3 bg-orange-50 dark:bg-orange-900/20 rounded-lg">
                    <p className="text-xs sm:text-sm text-muted-foreground">Length of Stay</p>
                    <p className="text-lg sm:text-xl font-bold text-orange-600">2.8 days</p>
                    <p className="text-xs text-green-600">+0.3 vs last month</p>
                  </div>
                </div>
                <div className="space-y-2 sm:space-y-3">
                  <div className="flex justify-between items-center">
                    <span className="text-xs sm:text-sm">Occupancy Rate</span>
                    <span className="text-xs sm:text-sm font-medium">78.5%</span>
                  </div>
                  <Progress value={78.5} className="h-2" />
                  <div className="flex justify-between items-center">
                    <span className="text-xs sm:text-sm">Revenue Efficiency</span>
                    <span className="text-xs sm:text-sm font-medium">92.3%</span>
                  </div>
                  <Progress value={92.3} className="h-2" />
                  <div className="flex justify-between items-center">
                    <span className="text-xs sm:text-sm">Guest Satisfaction</span>
                    <span className="text-xs sm:text-sm font-medium">94.0%</span>
                  </div>
                  <Progress value={94.0} className="h-2" />
                </div>
              </CardContent>
            </Card>
          </div>
        </TabsContent>

            <TabsContent value="realtime" className="space-y-3 sm:space-y-4 h-full">
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-3 sm:gap-4 md:gap-6">
            {/* Live Activity Feed */}
            <Card className="lg:col-span-2 min-h-[400px] sm:min-h-[450px] md:min-h-[500px]">
              <CardHeader className="pb-2 sm:pb-3 md:pb-4">
                <CardTitle className="text-sm sm:text-lg md:text-xl leading-tight">Live Activity Feed</CardTitle>
                <CardDescription className="text-xs sm:text-sm leading-tight">Real-time hotel activities and events</CardDescription>
              </CardHeader>
              <CardContent className="h-full">
                <div className="space-y-2 sm:space-y-3 h-full overflow-y-auto">
                  <div className="flex items-center space-x-2 sm:space-x-3 p-2 sm:p-3 bg-green-50 dark:bg-green-900/20 rounded-lg">
                    <div className="w-2.5 h-2.5 sm:w-3 sm:h-3 bg-green-500 rounded-full animate-pulse flex-shrink-0"></div>
                    <div className="flex-1">
                      <p className="text-xs sm:text-sm font-medium">Guest Check-in: Room 205</p>
                      <p className="text-xs text-muted-foreground">Sarah Johnson - 2 minutes ago</p>
                    </div>
                  </div>
                  <div className="flex items-center space-x-2 sm:space-x-3 p-2 sm:p-3 bg-blue-50 dark:bg-blue-900/20 rounded-lg">
                    <div className="w-2.5 h-2.5 sm:w-3 sm:h-3 bg-blue-500 rounded-full animate-pulse flex-shrink-0"></div>
                    <div className="flex-1">
                      <p className="text-xs sm:text-sm font-medium">Key Card Issued: Room 312</p>
                      <p className="text-xs text-muted-foreground">Mike Wilson - 5 minutes ago</p>
                    </div>
                  </div>
                  <div className="flex items-center space-x-2 sm:space-x-3 p-2 sm:p-3 bg-purple-50 dark:bg-purple-900/20 rounded-lg">
                    <div className="w-2.5 h-2.5 sm:w-3 sm:h-3 bg-purple-500 rounded-full animate-pulse flex-shrink-0"></div>
                    <div className="flex-1">
                      <p className="text-xs sm:text-sm font-medium">Service Request: Room 101</p>
                      <p className="text-xs text-muted-foreground">Housekeeping - 8 minutes ago</p>
                    </div>
                  </div>
                  <div className="flex items-center space-x-2 sm:space-x-3 p-2 sm:p-3 bg-orange-50 dark:bg-orange-900/20 rounded-lg">
                    <div className="w-2.5 h-2.5 sm:w-3 sm:h-3 bg-orange-500 rounded-full animate-pulse flex-shrink-0"></div>
                    <div className="flex-1">
                      <p className="text-xs sm:text-sm font-medium">Guest Check-out: Room 156</p>
                      <p className="text-xs text-muted-foreground">David Brown - 12 minutes ago</p>
                    </div>
                  </div>
                  <div className="flex items-center space-x-2 sm:space-x-3 p-2 sm:p-3 bg-red-50 dark:bg-red-900/20 rounded-lg">
                    <div className="w-2.5 h-2.5 sm:w-3 sm:h-3 bg-red-500 rounded-full animate-pulse flex-shrink-0"></div>
                    <div className="flex-1">
                      <p className="text-xs sm:text-sm font-medium">Maintenance Alert: Room 89</p>
                      <p className="text-xs text-muted-foreground">AC System - 15 minutes ago</p>
                    </div>
                  </div>
                </div>
              </CardContent>
            </Card>

            {/* Live Metrics */}
            <Card className="min-h-[400px] sm:min-h-[450px] md:min-h-[500px]">
              <CardHeader className="pb-2 sm:pb-3 md:pb-4">
                <CardTitle className="text-sm sm:text-lg md:text-xl leading-tight">Live Metrics</CardTitle>
                <CardDescription className="text-xs sm:text-sm leading-tight">Current system status</CardDescription>
              </CardHeader>
              <CardContent className="space-y-3 sm:space-y-4">
                <div className="space-y-2 sm:space-y-3">
                  <div className="flex justify-between items-center">
                    <span className="text-xs sm:text-sm">Active Guests</span>
                    <span className="text-xs sm:text-sm font-medium">142</span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-xs sm:text-sm">Available Rooms</span>
                    <span className="text-xs sm:text-sm font-medium">58</span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-xs sm:text-sm">Staff Online</span>
                    <span className="text-xs sm:text-sm font-medium">23</span>
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-xs sm:text-sm">System Uptime</span>
                    <span className="text-xs sm:text-sm font-medium">99.8%</span>
                  </div>
                </div>
                <div className="pt-2 sm:pt-3 border-t">
                  <p className="text-xs sm:text-sm font-medium mb-2">System Status</p>
                  <div className="space-y-1 sm:space-y-2">
                    <div className="flex items-center space-x-2">
                      <div className="w-2.5 h-2.5 sm:w-3 sm:h-3 bg-green-500 rounded-full flex-shrink-0"></div>
                      <span className="text-xs sm:text-sm">RFID System</span>
                    </div>
                    <div className="flex items-center space-x-2">
                      <div className="w-2.5 h-2.5 sm:w-3 sm:h-3 bg-green-500 rounded-full flex-shrink-0"></div>
                      <span className="text-xs sm:text-sm">Security Cameras</span>
                    </div>
                    <div className="flex items-center space-x-2">
                      <div className="w-2.5 h-2.5 sm:w-3 sm:h-3 bg-green-500 rounded-full flex-shrink-0"></div>
                      <span className="text-xs sm:text-sm">HVAC Systems</span>
                    </div>
                    <div className="flex items-center space-x-2">
                      <div className="w-2.5 h-2.5 sm:w-3 sm:h-3 bg-yellow-500 rounded-full flex-shrink-0"></div>
                      <span className="text-xs sm:text-sm">WiFi Network</span>
                    </div>
                  </div>
                </div>
              </CardContent>
            </Card>
          </div>
        </TabsContent>

            <TabsContent value="occupancy" className="space-y-3 sm:space-y-4 h-full">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-3 sm:gap-4 md:gap-6">
            {/* Occupancy Analytics */}
            <Card className="min-h-[300px] sm:min-h-[350px] md:min-h-[400px]">
              <CardHeader className="pb-2 sm:pb-3">
                <div className="flex flex-col sm:flex-row items-start sm:items-center justify-between gap-3 sm:gap-4">
                  <div>
                    <CardTitle className="text-sm sm:text-lg leading-tight">Occupancy Analytics</CardTitle>
                    <CardDescription className="text-xs sm:text-sm leading-tight">Detailed occupancy performance metrics</CardDescription>
                  </div>
                  <Select value={selectedMetric} onValueChange={setSelectedMetric}>
                    <SelectTrigger className="w-full sm:w-[140px] md:w-[180px] text-xs sm:text-sm h-8 sm:h-10">
                      <SelectValue placeholder="Select Metric" />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="occupancy">Occupancy Rate</SelectItem>
                      <SelectItem value="revenue">Revenue</SelectItem>
                      <SelectItem value="guests">Guest Count</SelectItem>
                    </SelectContent>
                  </Select>
                </div>
              </CardHeader>
              <CardContent className="h-full">
                <div className="h-[250px] sm:h-[280px] md:h-[320px]">
                  <ChartContainer config={chartConfig}>
                    <ComposedChart data={occupancyData}>
                      <CartesianGrid strokeDasharray="3 3" className="stroke-muted" />
                      <XAxis dataKey="month" className="text-xs" />
                      <YAxis className="text-xs" />
                      <ChartTooltip content={<ChartTooltipContent />} />
                      <Area
                        type="monotone"
                        dataKey={selectedMetric}
                        fill="hsl(var(--chart-1))"
                        fillOpacity={0.1}
                        stroke="hsl(var(--chart-1))"
                        strokeWidth={2}
                      />
                      <Bar dataKey={selectedMetric} fill="hsl(var(--chart-1))" opacity={0.8} />
                    </ComposedChart>
                  </ChartContainer>
                </div>
              </CardContent>
            </Card>

            {/* Occupancy Details */}
            <Card className="min-h-[300px] sm:min-h-[350px] md:min-h-[400px]">
              <CardHeader className="pb-2 sm:pb-3">
                <CardTitle className="text-sm sm:text-lg leading-tight">Occupancy Details</CardTitle>
                <CardDescription className="text-xs sm:text-sm leading-tight">Room type occupancy breakdown</CardDescription>
              </CardHeader>
              <CardContent className="space-y-3 sm:space-y-4">
                <div className="space-y-2 sm:space-y-3">
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-blue-50 dark:bg-blue-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Standard Rooms</p>
                      <p className="text-xs text-muted-foreground">45 rooms total</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">82%</p>
                      <p className="text-xs text-green-600">+5.2%</p>
                    </div>
                  </div>
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-green-50 dark:bg-green-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Deluxe Rooms</p>
                      <p className="text-xs text-muted-foreground">30 rooms total</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">91%</p>
                      <p className="text-xs text-green-600">+8.7%</p>
                    </div>
                  </div>
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-purple-50 dark:bg-purple-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Suite Rooms</p>
                      <p className="text-xs text-muted-foreground">15 rooms total</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">67%</p>
                      <p className="text-xs text-red-600">-2.1%</p>
                    </div>
                  </div>
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-orange-50 dark:bg-orange-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Presidential</p>
                      <p className="text-xs text-muted-foreground">10 rooms total</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">100%</p>
                      <p className="text-xs text-green-600">+0.0%</p>
                    </div>
                  </div>
                </div>
                <div className="pt-2 sm:pt-3 border-t">
                  <p className="text-xs sm:text-sm font-medium mb-2">Occupancy Trends</p>
                  <div className="space-y-1 sm:space-y-2">
                    <div className="flex justify-between items-center">
                      <span className="text-xs sm:text-sm">Peak Hours</span>
                      <span className="text-xs sm:text-sm">2:00 PM - 6:00 PM</span>
                    </div>
                    <div className="flex justify-between items-center">
                      <span className="text-xs sm:text-sm">Lowest Hours</span>
                      <span className="text-xs sm:text-sm">2:00 AM - 6:00 AM</span>
                    </div>
                    <div className="flex justify-between items-center">
                      <span className="text-xs sm:text-sm">Avg Stay Duration</span>
                      <span className="text-xs sm:text-sm">2.8 days</span>
                    </div>
                  </div>
                </div>
              </CardContent>
            </Card>
          </div>
        </TabsContent>

            <TabsContent value="revenue" className="space-y-3 sm:space-y-4 h-full">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-3 sm:gap-4 md:gap-6">
            {/* Revenue Analytics */}
            <Card className="min-h-[300px] sm:min-h-[350px] md:min-h-[400px]">
              <CardHeader className="pb-2 sm:pb-3">
                <CardTitle className="text-sm sm:text-lg leading-tight">Revenue Analytics</CardTitle>
                <CardDescription className="text-xs sm:text-sm leading-tight">Revenue trends and performance</CardDescription>
              </CardHeader>
              <CardContent className="h-full">
                <div className="h-[250px] sm:h-[280px] md:h-[320px]">
                  <ChartContainer config={chartConfig}>
                    <AreaChart data={occupancyData}>
                      <CartesianGrid strokeDasharray="3 3" className="stroke-muted" />
                      <XAxis dataKey="month" className="text-xs" />
                      <YAxis className="text-xs" />
                      <ChartTooltip content={<ChartTooltipContent />} />
                      <Area
                        type="monotone"
                        dataKey="revenue"
                        fill="hsl(var(--chart-2))"
                        fillOpacity={0.3}
                        stroke="hsl(var(--chart-2))"
                        strokeWidth={2}
                      />
                    </AreaChart>
                  </ChartContainer>
                </div>
              </CardContent>
            </Card>

            {/* Revenue Breakdown */}
            <Card className="min-h-[300px] sm:min-h-[350px] md:min-h-[400px]">
              <CardHeader className="pb-2 sm:pb-3">
                <CardTitle className="text-sm sm:text-lg leading-tight">Revenue Breakdown</CardTitle>
                <CardDescription className="text-xs sm:text-sm leading-tight">Revenue by category and source</CardDescription>
              </CardHeader>
              <CardContent className="space-y-3 sm:space-y-4">
                <div className="space-y-2 sm:space-y-3">
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-green-50 dark:bg-green-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Room Revenue</p>
                      <p className="text-xs text-muted-foreground">Accommodation charges</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">$52,450</p>
                      <p className="text-xs text-green-600">72.5%</p>
                    </div>
                  </div>
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-blue-50 dark:bg-blue-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Food & Beverage</p>
                      <p className="text-xs text-muted-foreground">Restaurant and bar</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">$12,800</p>
                      <p className="text-xs text-green-600">17.7%</p>
                    </div>
                  </div>
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-purple-50 dark:bg-purple-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Spa & Wellness</p>
                      <p className="text-xs text-muted-foreground">Spa services</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">$4,200</p>
                      <p className="text-xs text-green-600">5.8%</p>
                    </div>
                  </div>
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-orange-50 dark:bg-orange-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Other Services</p>
                      <p className="text-xs text-muted-foreground">Laundry, parking, etc.</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">$3,000</p>
                      <p className="text-xs text-green-600">4.1%</p>
                    </div>
                  </div>
                </div>
                <div className="pt-2 sm:pt-3 border-t">
                  <p className="text-xs sm:text-sm font-medium mb-2">Revenue Metrics</p>
                  <div className="space-y-1 sm:space-y-2">
                    <div className="flex justify-between items-center">
                      <span className="text-xs sm:text-sm">RevPAR</span>
                      <span className="text-xs sm:text-sm font-medium">$189.50</span>
                    </div>
                    <div className="flex justify-between items-center">
                      <span className="text-xs sm:text-sm">ADR</span>
                      <span className="text-xs sm:text-sm font-medium">$243.20</span>
                    </div>
                    <div className="flex justify-between items-center">
                      <span className="text-xs sm:text-sm">Profit Margin</span>
                      <span className="text-xs sm:text-sm font-medium">68.5%</span>
                    </div>
                  </div>
                </div>
              </CardContent>
            </Card>
          </div>
        </TabsContent>

            <TabsContent value="guests" className="space-y-3 sm:space-y-4 h-full">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-3 sm:gap-4 md:gap-6">
            {/* Guest Analytics */}
            <Card className="min-h-[300px] sm:min-h-[350px] md:min-h-[400px]">
              <CardHeader className="pb-2 sm:pb-3">
                <CardTitle className="text-sm sm:text-lg leading-tight">Guest Analytics</CardTitle>
                <CardDescription className="text-xs sm:text-sm leading-tight">Guest count and demographics</CardDescription>
              </CardHeader>
              <CardContent className="h-full">
                <div className="h-[250px] sm:h-[280px] md:h-[320px]">
                  <ChartContainer config={chartConfig}>
                    <BarChart data={occupancyData}>
                      <CartesianGrid strokeDasharray="3 3" className="stroke-muted" />
                      <XAxis dataKey="month" className="text-xs" />
                      <YAxis className="text-xs" />
                      <ChartTooltip content={<ChartTooltipContent />} />
                      <Bar dataKey="guests" fill="hsl(var(--chart-3))" />
                    </BarChart>
                  </ChartContainer>
                </div>
              </CardContent>
            </Card>

            {/* Guest Demographics */}
            <Card className="min-h-[300px] sm:min-h-[350px] md:min-h-[400px]">
              <CardHeader className="pb-2 sm:pb-3">
                <CardTitle className="text-sm sm:text-lg leading-tight">Guest Demographics</CardTitle>
                <CardDescription className="text-xs sm:text-sm leading-tight">Guest profile and preferences</CardDescription>
              </CardHeader>
              <CardContent className="space-y-3 sm:space-y-4">
                <div className="space-y-2 sm:space-y-3">
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-blue-50 dark:bg-blue-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Business Travelers</p>
                      <p className="text-xs text-muted-foreground">Corporate guests</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">45%</p>
                      <p className="text-xs text-green-600">+3.2%</p>
                    </div>
                  </div>
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-green-50 dark:bg-green-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Leisure Travelers</p>
                      <p className="text-xs text-muted-foreground">Vacation guests</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">38%</p>
                      <p className="text-xs text-green-600">+1.8%</p>
                    </div>
                  </div>
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-purple-50 dark:bg-purple-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">International</p>
                      <p className="text-xs text-muted-foreground">Foreign guests</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">17%</p>
                      <p className="text-xs text-green-600">+5.4%</p>
                    </div>
                  </div>
                </div>
                <div className="pt-2 sm:pt-3 border-t">
                  <p className="text-xs sm:text-sm font-medium mb-2">Guest Insights</p>
                  <div className="space-y-1 sm:space-y-2">
                    <div className="flex justify-between items-center">
                      <span className="text-xs sm:text-sm">Avg Age</span>
                      <span className="text-xs sm:text-sm">42 years</span>
                    </div>
                    <div className="flex justify-between items-center">
                      <span className="text-xs sm:text-sm">Repeat Guests</span>
                      <span className="text-xs sm:text-sm">34%</span>
                    </div>
                    <div className="flex justify-between items-center">
                      <span className="text-xs sm:text-sm">Booking Lead Time</span>
                      <span className="text-xs sm:text-sm">12.5 days</span>
                    </div>
                  </div>
                </div>
              </CardContent>
            </Card>
          </div>
        </TabsContent>

            <TabsContent value="services" className="space-y-3 sm:space-y-4 h-full">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-3 sm:gap-4 md:gap-6">
            {/* Service Ratings */}
            <Card className="min-h-[300px] sm:min-h-[350px] md:min-h-[400px]">
              <CardHeader className="pb-2 sm:pb-3">
                <CardTitle className="text-sm sm:text-lg leading-tight">Service Ratings</CardTitle>
                <CardDescription className="text-xs sm:text-sm leading-tight">Guest satisfaction ratings by service category</CardDescription>
              </CardHeader>
              <CardContent className="h-full">
                <div className="h-[250px] sm:h-[280px] md:h-[320px] flex items-center justify-center">
                  <ChartContainer config={chartConfig}>
                    <RadarChart data={serviceRatings}>
                      <PolarGrid />
                      <PolarAngleAxis dataKey="service" />
                      <PolarRadiusAxis angle={90} domain={[0, 5]} />
                      <Radar
                        name="Current Rating"
                        dataKey="rating"
                        stroke="#3b82f6"
                        fill="#3b82f6"
                        fillOpacity={0.3}
                      />
                      <Radar
                        name="Target Rating"
                        dataKey="target"
                        stroke="#ef4444"
                        fill="#ef4444"
                        fillOpacity={0.1}
                      />
                    </RadarChart>
                  </ChartContainer>
                </div>
              </CardContent>
            </Card>

            {/* Service Details */}
            <Card className="min-h-[300px] sm:min-h-[350px] md:min-h-[400px]">
              <CardHeader className="pb-2 sm:pb-3">
                <CardTitle className="text-sm sm:text-lg leading-tight">Service Details</CardTitle>
                <CardDescription className="text-xs sm:text-sm leading-tight">Detailed service performance metrics</CardDescription>
              </CardHeader>
              <CardContent className="space-y-3 sm:space-y-4">
                <div className="space-y-2 sm:space-y-3">
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-green-50 dark:bg-green-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Cleanliness</p>
                      <p className="text-xs text-muted-foreground">Room and facility cleanliness</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">4.8/5.0</p>
                      <p className="text-xs text-green-600">+0.2</p>
                    </div>
                  </div>
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-blue-50 dark:bg-blue-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Staff Service</p>
                      <p className="text-xs text-muted-foreground">Customer service quality</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">4.6/5.0</p>
                      <p className="text-xs text-green-600">+0.1</p>
                    </div>
                  </div>
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-purple-50 dark:bg-purple-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Room Comfort</p>
                      <p className="text-xs text-muted-foreground">Room amenities and comfort</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">4.7/5.0</p>
                      <p className="text-xs text-green-600">+0.3</p>
                    </div>
                  </div>
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-orange-50 dark:bg-orange-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Food Quality</p>
                      <p className="text-xs text-muted-foreground">Restaurant and dining</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">4.5/5.0</p>
                      <p className="text-xs text-red-600">-0.1</p>
                    </div>
                  </div>
                </div>
                <div className="pt-2 sm:pt-3 border-t">
                  <p className="text-xs sm:text-sm font-medium mb-2">Service Metrics</p>
                  <div className="space-y-1 sm:space-y-2">
                    <div className="flex justify-between items-center">
                      <span className="text-xs sm:text-sm">Response Time</span>
                      <span className="text-xs sm:text-sm">2.3 minutes</span>
                    </div>
                    <div className="flex justify-between items-center">
                      <span className="text-xs sm:text-sm">Resolution Rate</span>
                      <span className="text-xs sm:text-sm">96.8%</span>
                    </div>
                    <div className="flex justify-between items-center">
                      <span className="text-xs sm:text-sm">Guest Satisfaction</span>
                      <span className="text-xs sm:text-sm">94.2%</span>
                    </div>
                  </div>
                </div>
              </CardContent>
            </Card>
          </div>
        </TabsContent>

            <TabsContent value="activity" className="space-y-3 sm:space-y-4 h-full">
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-3 sm:gap-4 md:gap-6">
            {/* Daily Activity Patterns */}
            <Card className="min-h-[300px] sm:min-h-[350px] md:min-h-[400px]">
              <CardHeader className="pb-2 sm:pb-3">
                <CardTitle className="text-sm sm:text-lg leading-tight">Daily Activity Patterns</CardTitle>
                <CardDescription className="text-xs sm:text-sm leading-tight">Hotel activity throughout the day</CardDescription>
              </CardHeader>
              <CardContent className="h-full">
                <div className="h-[250px] sm:h-[280px] md:h-[320px]">
                  <ChartContainer config={chartConfig}>
                    <ComposedChart data={dailyActivityData}>
                      <CartesianGrid strokeDasharray="3 3" className="stroke-muted" />
                      <XAxis dataKey="hour" className="text-xs" />
                      <YAxis className="text-xs" />
                      <ChartTooltip content={<ChartTooltipContent />} />
                      <Bar dataKey="checkins" fill="#3b82f6" name="Check-ins" />
                      <Bar dataKey="checkouts" fill="#ef4444" name="Check-outs" />
                      <Line
                        type="monotone"
                        dataKey="services"
                        stroke="#10b981"
                        strokeWidth={2}
                        name="Service Requests"
                      />
                    </ComposedChart>
                  </ChartContainer>
                </div>
              </CardContent>
            </Card>

            {/* Activity Summary */}
            <Card className="min-h-[300px] sm:min-h-[350px] md:min-h-[400px]">
              <CardHeader className="pb-2 sm:pb-3">
                <CardTitle className="text-sm sm:text-lg leading-tight">Activity Summary</CardTitle>
                <CardDescription className="text-xs sm:text-sm leading-tight">Today's activity breakdown</CardDescription>
              </CardHeader>
              <CardContent className="space-y-3 sm:space-y-4">
                <div className="space-y-2 sm:space-y-3">
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-blue-50 dark:bg-blue-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Check-ins</p>
                      <p className="text-xs text-muted-foreground">Guest arrivals</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">23</p>
                      <p className="text-xs text-green-600">+15%</p>
                    </div>
                  </div>
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-red-50 dark:bg-red-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Check-outs</p>
                      <p className="text-xs text-muted-foreground">Guest departures</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">18</p>
                      <p className="text-xs text-green-600">+8%</p>
                    </div>
                  </div>
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-green-50 dark:bg-green-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Service Requests</p>
                      <p className="text-xs text-muted-foreground">Guest services</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">45</p>
                      <p className="text-xs text-green-600">+22%</p>
                    </div>
                  </div>
                  <div className="flex justify-between items-center p-2 sm:p-3 bg-purple-50 dark:bg-purple-900/20 rounded-lg">
                    <div>
                      <p className="text-xs sm:text-sm font-medium">Key Cards Issued</p>
                      <p className="text-xs text-muted-foreground">New card activations</p>
                    </div>
                    <div className="text-right">
                      <p className="text-sm sm:text-lg font-bold">12</p>
                      <p className="text-xs text-green-600">+5%</p>
                    </div>
                  </div>
                </div>
                <div className="pt-2 sm:pt-3 border-t">
                  <p className="text-xs sm:text-sm font-medium mb-2">Peak Activity Times</p>
                  <div className="space-y-1 sm:space-y-2">
                    <div className="flex justify-between items-center">
                      <span className="text-xs sm:text-sm">Check-in Peak</span>
                      <span className="text-xs sm:text-sm">2:00 PM - 4:00 PM</span>
                    </div>
                    <div className="flex justify-between items-center">
                      <span className="text-xs sm:text-sm">Check-out Peak</span>
                      <span className="text-xs sm:text-sm">10:00 AM - 12:00 PM</span>
                    </div>
                    <div className="flex justify-between items-center">
                      <span className="text-xs sm:text-sm">Service Peak</span>
                      <span className="text-xs sm:text-sm">6:00 PM - 8:00 PM</span>
                    </div>
                  </div>
                </div>
              </CardContent>
            </Card>
          </div>
            </TabsContent>
          </div>
        </Tabs>
      </div>
    </div>
  )
}