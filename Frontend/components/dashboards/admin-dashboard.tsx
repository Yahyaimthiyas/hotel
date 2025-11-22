"use client"

import { useState, useEffect, useRef } from "react"
import { HotelsOverview } from "./hotels-overview"
import { HotelDashboard } from "./hotel-dashboard"

interface AdminDashboardProps {
  onHotelViewChange?: (isHotelView: boolean) => void
}

export function AdminDashboard({ onHotelViewChange }: AdminDashboardProps) {
  const [selectedHotelId, setSelectedHotelId] = useState<string | null>(null)
  const hotelsOverviewRef = useRef<{ refreshHotels?: () => void }>(null)

  const handleHotelSelect = (hotelId: string) => {
    setSelectedHotelId(hotelId)
    onHotelViewChange?.(true)
  }

  const handleBackToHotels = () => {
    setSelectedHotelId(null)
    onHotelViewChange?.(false)
    // Refresh hotels data when returning to overview
    setTimeout(() => {
      hotelsOverviewRef.current?.refreshHotels?.()
    }, 100)
  }

  if (selectedHotelId) {
    return <HotelDashboard hotelId={selectedHotelId} onBackToHotels={handleBackToHotels} />
  }

  return <HotelsOverview ref={hotelsOverviewRef} onHotelSelect={handleHotelSelect} />
}
