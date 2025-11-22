"use client"

/**
 * @file room-management.tsx
 * @brief Room management dashboard for a single hotel, including
 *        filtering, sorting, power statistics, and room details.
 */

import { useState, useMemo } from "react"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { Avatar, AvatarFallback, AvatarImage } from "@/components/ui/avatar"
import { Progress } from "@/components/ui/progress"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import { RoomCard } from "@/components/ui/room-card"
import { useHotelData } from "@/hooks/useHotelData"
import type { Settings as HotelSettings } from "@/lib/api"

import { 
  Building2, 
  Users, 
  Bed, 
  Search, 
  Filter, 
  SortAsc, 
  SortDesc,
  Calendar,
  Clock,
  User,
  Shield,
  Eye,
  Settings,
  CheckCircle,
  Zap
} from "lucide-react"

/**
 * Props for the RoomManagement dashboard.
 * @property hotelId Selected hotel identifier as string.
 */
interface RoomManagementProps {
  hotelId: string
  settings: HotelSettings | null
  updateSettings: (data: Partial<HotelSettings>) => Promise<void>
}

/**
 * Room-level management view for a hotel. Uses live room and power data
 * from the backend and offers filters, sorting, and a details drawer.
 */
export function RoomManagement({ hotelId, settings, updateSettings }: RoomManagementProps) {
  const { rooms, loading, error, powerReadings, activities } = useHotelData(hotelId)
  const [searchTerm, setSearchTerm] = useState("")
  const [floorFilter, setFloorFilter] = useState("all")
  const [typeFilter, setTypeFilter] = useState("all")
  const [occupancyFilter, setOccupancyFilter] = useState("all")
  const [occupantTypeFilter, setOccupantTypeFilter] = useState("all")
  const [sortBy, setSortBy] = useState("number")
  const [sortOrder, setSortOrder] = useState<"asc" | "desc">("asc")
  const [selectedRoomNumber, setSelectedRoomNumber] = useState<string | null>(null)

  // Transform live room data to match the expected format
  const transformedRooms = useMemo(() => {
    if (!rooms) return []

    // Build a lookup of latest activity text per room number
    const latestActivityByRoom = new Map<string, string>()

    if (activities && activities.length > 0) {
      for (const activity of activities) {
        const match = activity.action.match(/Room\s+(\d+)/)
        if (!match) continue
        const roomNum = match[1]
        if (!latestActivityByRoom.has(roomNum)) {
          latestActivityByRoom.set(roomNum, activity.action)
        }
      }
    }
    
    return rooms.map(room => {
      const status = room.status || "vacant"
      const lastActivity = latestActivityByRoom.get(room.number) || "N/A"
      return {
        id: room.id.toString(),
        number: room.number,
        type: "Standard", // Default type since it's not in the database
        floor: room.number.charAt(0), // Extract floor from room number
        occupancy: status,
        occupant: room.occupantType ? {
          name: room.occupantType,
          type: room.occupantType,
          avatar: "/placeholder.svg?height=40&width=40"
        } : undefined,
        lastActivity,
        status,
      }
    })
  }, [rooms, activities])

  /** Apply text filters, floor/type filters, and sorting to rooms. */
  const filteredAndSortedRooms = useMemo(() => {
    let filtered = transformedRooms.filter((room) => {
      const matchesSearch = 
        room.number.toLowerCase().includes(searchTerm.toLowerCase()) ||
        (room.occupant?.name.toLowerCase().includes(searchTerm.toLowerCase()) || false)
      const matchesFloor = floorFilter === "all" || room.floor === floorFilter
      const matchesType = typeFilter === "all" || room.type === typeFilter
      const matchesOccupancy = occupancyFilter === "all" || room.occupancy === occupancyFilter
      const matchesOccupantType = occupantTypeFilter === "all" || room.occupant?.type === occupantTypeFilter || (occupantTypeFilter === "maintenance" && room.status === "maintenance")
      
      return matchesSearch && matchesFloor && matchesType && matchesOccupancy && matchesOccupantType
    })

    filtered.sort((a, b) => {
      let aValue, bValue
      
      switch (sortBy) {
        case "number":
          aValue = parseInt(a.number)
          bValue = parseInt(b.number)
          break
        case "floor":
          aValue = parseInt(a.floor)
          bValue = parseInt(b.floor)
          break
        case "occupancy":
          aValue = a.occupancy
          bValue = b.occupancy
          break
        case "lastActivity":
          aValue = a.lastActivity
          bValue = b.lastActivity
          break
        default:
          aValue = a.number
          bValue = b.number
      }
      
      if (sortOrder === "asc") {
        return aValue > bValue ? 1 : -1
      } else {
        return aValue < bValue ? 1 : -1
      }
    })

    return filtered
  }, [transformedRooms, searchTerm, floorFilter, typeFilter, occupancyFilter, occupantTypeFilter, sortBy, sortOrder])

  /** Aggregate basic room occupancy statistics. */
  const roomStats = useMemo(() => {
    const total = transformedRooms.length
    const occupied = transformedRooms.filter(room => room.occupancy === "occupied").length
    const vacant = transformedRooms.filter(room => room.occupancy === "vacant").length
    const maintenance = transformedRooms.filter(room => 
      room.status === "maintenance" || room.status === "cleaning"
    ).length
    
    return { total, occupied, vacant, maintenance }
  }, [transformedRooms])

  /** Aggregate power-related statistics from room and reading data. */
  const powerStats = useMemo(() => {
    const totalRooms = rooms.length
    const powerOn = rooms.filter(room => room.powerStatus === "on").length
    const powerOff = totalRooms - powerOn
    const lowPower = rooms.filter(room => room.hasLowPower).length

    return { totalRooms, powerOn, powerOff, lowPower, readingCount: powerReadings.length }
  }, [rooms, powerReadings])

  /** Unique floor list for floor filter dropdown. */
  const floors = useMemo(() => {
    const uniqueFloors = [...new Set(transformedRooms.map(room => room.floor))]
    return uniqueFloors.sort()
  }, [transformedRooms])

  /** Unique room type list for type filter dropdown. */
  const roomTypes = useMemo(() => {
    const uniqueTypes = [...new Set(transformedRooms.map(room => room.type))]
    return uniqueTypes.sort()
  }, [transformedRooms])

  /**
   * Selected room details including backend room record and most recent
   * non-stale power reading for the side panel.
   */
  const selectedRoom = useMemo(() => {
    if (!selectedRoomNumber) return null
    const base = transformedRooms.find(room => room.number === selectedRoomNumber)
    if (!base) return null
    const backendRoom = rooms.find(room => room.number === selectedRoomNumber)

    let power = powerReadings.find(reading => reading.room === selectedRoomNumber)
    if (power) {
      const ts = new Date(power.timestamp)
      const isInvalid = Number.isNaN(ts.getTime())
      const ageSeconds = isInvalid ? Infinity : (Date.now() - ts.getTime()) / 1000
      const isStale = ageSeconds > 60
      if (isInvalid || isStale) {
        power = undefined
      }
    }

    return { base, backendRoom, power }
  }, [selectedRoomNumber, transformedRooms, rooms, powerReadings])

  if (loading) {
    return (
      <div className="space-y-6">
        <div className="text-center py-8">
          <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-primary mx-auto mb-4"></div>
          <p className="text-muted-foreground">Loading rooms...</p>
        </div>
      </div>
    )
  }

  if (error) {
    return (
      <div className="space-y-6">
        <div className="text-center py-8">
          <div className="text-red-500 mb-4">
            <Building2 className="h-12 w-12 mx-auto" />
          </div>
          <h2 className="text-xl font-semibold mb-2">Error Loading Rooms</h2>
          <p className="text-muted-foreground mb-4">{error}</p>
        </div>
      </div>
    )
  }

  return (
    <div className="space-y-6">
      {/* Room Statistics */}
      <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4">
        <Card>
          <CardContent className="p-4">
            <div className="flex items-center justify-between">
              <div>
                <p className="text-sm font-medium text-muted-foreground">Total Rooms</p>
                <p className="text-2xl font-bold">{roomStats.total}</p>
              </div>
              <Building2 className="h-8 w-8 text-muted-foreground" />
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardContent className="p-4">
            <div className="flex items-center justify-between">
              <div>
                <p className="text-sm font-medium text-muted-foreground">Occupied</p>
                <p className="text-2xl font-bold text-red-600">{roomStats.occupied}</p>
              </div>
              <Users className="h-8 w-8 text-red-600" />
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardContent className="p-4">
            <div className="flex items-center justify-between">
              <div>
                <p className="text-sm font-medium text-muted-foreground">Vacant</p>
                <p className="text-2xl font-bold text-green-600">{roomStats.vacant}</p>
              </div>
              <Bed className="h-8 w-8 text-green-600" />
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardContent className="p-4">
            <div className="flex items-center justify-between">
              <div>
                <p className="text-sm font-medium text-muted-foreground">Maintenance</p>
                <p className="text-2xl font-bold text-yellow-500">{roomStats.maintenance}</p>
              </div>
              <Settings className="h-8 w-8 text-yellow-500" />
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardContent className="p-4">
            <div className="flex items-center justify-between">
              <div>
                <p className="text-sm font-medium text-muted-foreground">Power Status</p>
                <p className="text-xs text-muted-foreground">
                  {powerStats.powerOn} on · {powerStats.powerOff} off · {powerStats.lowPower} low power
                </p>
              </div>
              <Zap className="h-8 w-8 text-yellow-500" />
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Settings */}
      {settings && (
        <Card>
          <CardHeader>
            <CardTitle>Room Logic Settings</CardTitle>
            <CardDescription>
              Configure cleaning duration and low power threshold for this hotel.
            </CardDescription>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
              <div className="space-y-1">
                <label className="text-xs font-medium text-muted-foreground">
                  Minimum Cleaning Duration (minutes)
                </label>
                <Input
                  type="number"
                  defaultValue={Math.round(settings.minCleaningDurationSeconds / 60)}
                  onBlur={(e) => {
                    const minutes = parseInt(e.target.value || "0", 10)
                    if (!Number.isNaN(minutes) && minutes > 0) {
                      updateSettings({ minCleaningDurationSeconds: minutes * 60 })
                    }
                  }}
                />
              </div>
              <div className="space-y-1">
                <label className="text-xs font-medium text-muted-foreground">
                  Low Power Threshold (A)
                </label>
                <Input
                  type="number"
                  step="0.01"
                  defaultValue={settings.lowPowerCurrentThreshold}
                  onBlur={(e) => {
                    const value = parseFloat(e.target.value || "0")
                    if (!Number.isNaN(value) && value >= 0) {
                      updateSettings({ lowPowerCurrentThreshold: value })
                    }
                  }}
                />
              </div>
            </div>
          </CardContent>
        </Card>
      )}

      {/* Filters and Search */}
      <Card>
        <CardHeader>
          <CardTitle>Room Management</CardTitle>
          <CardDescription>Manage and monitor hotel rooms</CardDescription>
        </CardHeader>
        <CardContent className="space-y-4">
          <div className="flex flex-col sm:flex-row gap-4">
            <div className="relative flex-1">
              <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-muted-foreground" />
              <Input
                placeholder="Search rooms or occupants..."
                value={searchTerm}
                onChange={(e) => setSearchTerm(e.target.value)}
                className="pl-10"
              />
            </div>
            
            <Select value={floorFilter} onValueChange={setFloorFilter}>
              <SelectTrigger className="w-full sm:w-[140px]">
                <SelectValue placeholder="Floor" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="all">All Floors</SelectItem>
                {floors.map(floor => (
                  <SelectItem key={floor} value={floor}>Floor {floor}</SelectItem>
                ))}
              </SelectContent>
            </Select>

            <Select value={typeFilter} onValueChange={setTypeFilter}>
              <SelectTrigger className="w-full sm:w-[140px]">
                <SelectValue placeholder="Type" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="all">All Types</SelectItem>
                {roomTypes.map(type => (
                  <SelectItem key={type} value={type}>{type}</SelectItem>
                ))}
              </SelectContent>
            </Select>

            <Select value={occupancyFilter} onValueChange={setOccupancyFilter}>
              <SelectTrigger className="w-full sm:w-[140px]">
                <SelectValue placeholder="Occupancy" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="all">All</SelectItem>
                <SelectItem value="occupied">Occupied</SelectItem>
                <SelectItem value="vacant">Vacant</SelectItem>
              </SelectContent>
            </Select>

            <Select value={occupantTypeFilter} onValueChange={setOccupantTypeFilter}>
              <SelectTrigger className="w-full sm:w-[140px]">
                <SelectValue placeholder="Occupant" />
              </SelectTrigger>
              <SelectContent>
                <SelectItem value="all">All</SelectItem>
                <SelectItem value="guest">Guest</SelectItem>
                <SelectItem value="manager">Manager</SelectItem>
                <SelectItem value="maintenance">Maintenance</SelectItem>
              </SelectContent>
            </Select>
          </div>

          <div className="flex flex-col sm:flex-row gap-4 items-center">
            <div className="flex items-center gap-2">
              <span className="text-sm text-muted-foreground">Sort by:</span>
              <Select value={sortBy} onValueChange={setSortBy}>
                <SelectTrigger className="w-[120px]">
                  <SelectValue />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="number">Number</SelectItem>
                  <SelectItem value="floor">Floor</SelectItem>
                  <SelectItem value="occupancy">Occupancy</SelectItem>
                  <SelectItem value="lastActivity">Activity</SelectItem>
                </SelectContent>
              </Select>
            </div>
            
            <Button
              variant="outline"
              size="sm"
              onClick={() => setSortOrder(sortOrder === "asc" ? "desc" : "asc")}
            >
              {sortOrder === "asc" ? <SortAsc className="h-4 w-4" /> : <SortDesc className="h-4 w-4" />}
            </Button>

            <div className="ml-auto text-sm text-muted-foreground">
              {filteredAndSortedRooms.length} of {roomStats.total} rooms
            </div>
          </div>
        </CardContent>
      </Card>

      {/* Rooms Grid */}
      <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-4">
        {filteredAndSortedRooms.map((room) => (
          <RoomCard
            key={room.id}
            room={room}
            onClick={() => {
              /* Handle room click */
              setSelectedRoomNumber(room.number)
            }}
          />
        ))}
      </div>

      {selectedRoom && (
        <Card>
          <CardHeader className="flex flex-row items-center justify-between">
            <div>
              <CardTitle>Room {selectedRoom.base.number} Details</CardTitle>
              <CardDescription className="capitalize">{selectedRoom.base.occupancy}</CardDescription>
            </div>
            <Button variant="outline" size="sm" onClick={() => setSelectedRoomNumber(null)}>
              Close
            </Button>
          </CardHeader>
          <CardContent className="space-y-2 text-sm">
            <div className="flex justify-between">
              <span className="text-muted-foreground">Occupant</span>
              <span className="font-medium capitalize">
                {selectedRoom.base.occupant?.type || selectedRoom.backendRoom?.occupantType || "None"}
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">Power</span>
              <span className="font-medium">
                {selectedRoom.backendRoom?.powerStatus || "unknown"}
                {selectedRoom.backendRoom?.hasLowPower ? " · low" : ""}
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">Current</span>
              <span className="font-medium">
                {selectedRoom.power ? `${selectedRoom.power.current.toFixed(2)} A` : "No data"}
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">Last reading</span>
              <span className="font-medium">
                {selectedRoom.power ? selectedRoom.power.timestamp : "No data"}
              </span>
            </div>
          </CardContent>
        </Card>
      )}

      {/* Empty State */}
      {filteredAndSortedRooms.length === 0 && (
        <Card>
          <CardContent className="text-center py-8">
            <Building2 className="h-12 w-12 text-muted-foreground mx-auto mb-4" />
            <h3 className="text-lg font-medium mb-2">No rooms found</h3>
            <p className="text-muted-foreground">
              Try adjusting your search criteria or filters.
            </p>
          </CardContent>
        </Card>
      )}
    </div>
  )
}