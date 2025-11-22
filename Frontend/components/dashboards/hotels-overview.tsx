"use client"

/**
 * @file hotels-overview.tsx
 * @brief Landing view showing all hotels with search and responsive grid.
 */

import { useState, useEffect, forwardRef, useImperativeHandle } from "react"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Search, Building2, Users, CreditCard, Activity, MapPin, Star } from "lucide-react"
import { HotelCard } from "@/components/ui/hotel-card"
import { useHotelsData } from "@/hooks/useHotelData"
import { useDeviceDetection } from "@/hooks/use-mobile"

/**
 * Props for HotelsOverview.
 * @property onHotelSelect Callback when a hotel card is selected.
 */
interface HotelsOverviewProps {
  onHotelSelect: (hotelId: string) => void
}

/**
 * Overview grid of all hotels, with search and stats, exposing a
 * refresh method to parent components via ref.
 */
export const HotelsOverview = forwardRef<{ refreshHotels?: () => void }, HotelsOverviewProps>(
  ({ onHotelSelect }, ref) => {
  const [searchTerm, setSearchTerm] = useState("")
  const [selectedHotel, setSelectedHotel] = useState<string | null>(null)
  const { hotels, loading, error, refreshHotels } = useHotelsData()
  const { deviceType, isMobile, isTablet, isDesktop, orientation, shouldShowMobileLayout } = useDeviceDetection()

  // Expose refresh function to parent component
  useImperativeHandle(ref, () => ({
    refreshHotels
  }))

  /** Filter hotels by name, location, or manager name. */
  const filteredHotels = hotels.filter((hotel) =>
    hotel.name.toLowerCase().includes(searchTerm.toLowerCase()) ||
    hotel.location.toLowerCase().includes(searchTerm.toLowerCase()) ||
    hotel.manager.name.toLowerCase().includes(searchTerm.toLowerCase())
  )

  /** Handle selection of a hotel card. */
  const handleHotelClick = (hotelId: string) => {
    setSelectedHotel(hotelId)
    onHotelSelect(hotelId)
  }

  /** Map hotel status to Tailwind color classes. */
  const getStatusColor = (status: string) => {
    switch (status) {
      case "active":
        return "bg-status-active"
      case "maintenance":
        return "bg-status-maintenance"
      case "inactive":
        return "bg-status-inactive"
      default:
        return "bg-muted-foreground"
    }
  }

  /** Human readable status label for a hotel. */
  const getStatusText = (status: string) => {
    switch (status) {
      case "active":
        return "Operational"
      case "maintenance":
        return "Maintenance"
      case "inactive":
        return "Inactive"
      default:
        return "Unknown"
    }
  }

  // Responsive grid configuration based on device type
  const getGridConfig = () => {
    if (isMobile) {
      return "grid-cols-1"
    } else if (isTablet) {
      return orientation === 'portrait' ? "grid-cols-1 sm:grid-cols-2" : "grid-cols-2 md:grid-cols-3"
    } else {
      return "grid-cols-1 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4 xl:grid-cols-5 2xl:grid-cols-6"
    }
  }

  // Show error state
  if (error) {
    return (
      <div className="space-y-4 xs:space-y-6 sm:space-y-8 relative h-full">
        <div className="text-center py-8 xs:py-12 sm:py-16">
          <div className="text-red-500 mb-4">
            <Building2 className="h-12 w-12 mx-auto" />
          </div>
          <h2 className="text-xl font-semibold mb-2">Error Loading Hotels</h2>
          <p className="text-muted-foreground mb-4">{error}</p>
          <Button onClick={refreshHotels}>
            Retry
          </Button>
        </div>
      </div>
    )
  }

  return (
    <div className="space-y-4 xs:space-y-6 sm:space-y-8 relative h-full">
      
      {/* Header - Enhanced responsive typography */}
      <div className="relative z-10 text-center space-y-3 xs:space-y-4 sm:space-y-6">
        {/* Main Title with responsive typography */}
        <div className="space-y-2 xs:space-y-3 sm:space-y-4">
          <h1 className="text-2xl xs:text-3xl sm:text-4xl md:text-5xl lg:text-6xl xl:text-7xl font-light tracking-tight text-foreground leading-tight">
            Hotel Management
          </h1>
        </div>
        
        {/* Subtitle with responsive typography */}
        <p className="text-sm xs:text-base sm:text-lg md:text-xl lg:text-2xl text-muted-foreground font-normal leading-relaxed max-w-2xl xs:max-w-3xl md:max-w-4xl mx-auto px-3 xs:px-4 sm:px-6">
          Select a hotel to manage its RFID key card system
        </p>
        
        {/* Subtle decorative element */}
        <div className="flex items-center justify-center space-x-2 text-muted-foreground">
          <div className="w-6 xs:w-8 sm:w-12 h-px bg-border"></div>
          <Building2 className="h-3 xs:h-4 sm:h-5 w-3 xs:w-4 sm:w-5" />
          <div className="w-6 xs:w-8 sm:w-12 h-px bg-border"></div>
        </div>
      </div>

      {/* Search and Stats - Enhanced responsive layout */}
      <div className="flex flex-col gap-2 xs:gap-3 sm:gap-4 relative z-10 max-w-xl xs:max-w-2xl md:max-w-3xl lg:max-w-4xl mx-auto">
        <div className="relative w-full">
          <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 h-3 xs:h-4 w-3 xs:w-4 text-muted-foreground" />
          <Input
            placeholder="Search hotels..."
            value={searchTerm}
            onChange={(e) => setSearchTerm(e.target.value)}
            variant="glass"
            className="pl-8 xs:pl-10 h-8 xs:h-9 sm:h-10 md:h-11 text-sm xs:text-base touch-target"
          />
        </div>
        <div className="flex flex-col xs:flex-row items-center justify-between gap-1 xs:gap-2 text-xs xs:text-sm text-muted-foreground">
          <span className="font-medium">{filteredHotels.length} hotels found</span>
          <div className="flex items-center space-x-1 xs:space-x-2">
            <div className="w-2 xs:w-2.5 h-2 xs:h-2.5 bg-status-active rounded-full shadow-sm"></div>
            <span className="font-medium">Operational</span>
          </div>
        </div>
      </div>

      {/* Hotels Grid - Enhanced responsive grid */}
      <div className={`grid ${getGridConfig()} gap-3 xs:gap-4 sm:gap-5 md:gap-6 lg:gap-8 relative z-10 w-full max-w-none`}>
        {filteredHotels.map((hotel) => (
          <HotelCard
            key={hotel.id}
            hotel={hotel}
            isSelected={selectedHotel === hotel.id}
            onClick={() => handleHotelClick(hotel.id)}
          />
        ))}
      </div>

      {/* Empty State - Enhanced responsive design */}
      {filteredHotels.length === 0 && (
        <div className="text-center py-8 xs:py-12 sm:py-16 max-w-sm xs:max-w-md md:max-w-lg mx-auto">
          <Building2 className="h-8 xs:h-10 sm:h-12 w-8 xs:w-10 sm:w-12 text-muted-foreground mx-auto mb-3 xs:mb-4" />
          <h3 className="text-base xs:text-lg sm:text-xl font-medium mb-2 xs:mb-3">No hotels found</h3>
          <p className="text-xs xs:text-sm text-muted-foreground px-4">
            Try adjusting your search terms or add a new hotel to get started.
          </p>
        </div>
      )}

      {/* Loading State - Enhanced responsive design */}
      {loading && (
        <div className="text-center py-8 xs:py-12 sm:py-16">
          <div className="animate-spin rounded-full h-8 xs:h-10 sm:h-12 w-8 xs:w-10 sm:w-12 border-b-2 border-primary mx-auto mb-3 xs:mb-4"></div>
          <p className="text-sm xs:text-base text-muted-foreground">Loading hotels...</p>
        </div>
      )}
    </div>
  )
}) 