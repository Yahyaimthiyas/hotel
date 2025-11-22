"use client"

import { useState } from "react"
import { 
  Building2, 
  MapPin, 
  Star, 
  Phone, 
  Mail, 
  Users, 
  CreditCard, 
  Activity,
  Settings,
  BarChart3,
  Key,
  Shield,
  Calendar,
  Clock,
  ArrowLeft,
  CheckCircle,
  AlertTriangle,
  Menu,
  X,
  Edit
} from "lucide-react"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Avatar, AvatarFallback, AvatarImage } from "@/components/ui/avatar"
import { Separator } from "@/components/ui/separator"
import { 
  Sidebar, 
  SidebarHeader, 
  SidebarContent, 
  SidebarNav, 
  SidebarNavItem, 
  SidebarNavLink 
} from "@/components/ui/sidebar"

interface HotelDetailsSidebarProps {
  hotel: {
    id: string
    name: string
    location: string
    manager: {
      name: string
      phone: string
      email: string
      status: string
    }
    totalRooms?: number
    activeRooms?: number
    totalCards?: number
    status: string
    occupancy?: number
    lastActivity: string
    image: string
    description: string
  }
  activeSection: string
  onSectionChange: (section: string) => void
  onBackToHotels: () => void
  onEditHotel?: () => void
  isOpen?: boolean
  onOpenChange?: (open: boolean) => void
}

export function HotelDetailsSidebar({
  hotel,
  activeSection,
  onSectionChange,
  onBackToHotels,
  onEditHotel,
  isOpen = false,
  onOpenChange
}: HotelDetailsSidebarProps) {
  const getStatusColor = (status: string) => {
    switch (status) {
      case "active":
        return "bg-green-500"
      case "inactive":
        return "bg-red-500"
      case "maintenance":
        return "bg-yellow-500"
      default:
        return "bg-gray-500"
    }
  }

  const getStatusText = (status: string) => {
    switch (status) {
      case "active":
        return "Operational"
      case "inactive":
        return "Offline"
      case "maintenance":
        return "Maintenance"
      default:
        return "Unknown"
    }
  }

  const navigationItems = [
    {
      id: "overview",
      label: "Overview",
      icon: Building2,
      description: "Hotel summary and key metrics"
    },
    {
      id: "rooms",
      label: "Room Management",
      icon: Key,
      description: "Manage rooms and key cards"
    },
    {
      id: "analytics",
      label: "Analytics",
      icon: BarChart3,
      description: "Performance insights and reports"
    },
    {
      id: "staff",
      label: "Staff Management",
      icon: Users,
      description: "Manage hotel staff and permissions"
    },
    {
      id: "calendar",
      label: "Calendar",
      icon: Calendar,
      description: "View hotel calendar"
    }
  ]

  const supportItems = [
    {
      id: "activity",
      label: "Recent Activity",
      icon: Activity,
      description: "Latest system activity"
    }
  ]

  return (
    <Sidebar isOpen={isOpen} onOpenChange={onOpenChange}>
      <SidebarHeader>
                  <div className="flex items-center space-x-2 px-2">
            <Button
              variant="ghost"
              size="icon"
              onClick={onBackToHotels}
              className="h-7 w-7 text-gray-500 hover:text-gray-700 dark:text-gray-400 dark:hover:text-gray-200 flex-shrink-0"
            >
              <ArrowLeft className="h-4 w-4" />
            </Button>
            <div className="w-7 h-7 rounded-lg bg-blue-600 flex items-center justify-center flex-shrink-0">
              <Building2 className="h-4 w-4 text-white" />
            </div>
            <div className="flex-1 min-w-0">
              <h2 className="text-sm font-semibold text-gray-900 dark:text-white truncate">RFID Manager</h2>
              <p className="text-xs text-gray-500 dark:text-gray-400 truncate">Hotel Key Card System</p>
            </div>
          <Button
            variant="ghost"
            size="icon"
            onClick={() => onOpenChange?.(false)}
            className="h-7 w-7 text-gray-500 hover:text-gray-700 dark:text-gray-400 dark:hover:text-gray-200 lg:hidden flex-shrink-0"
          >
            <X className="h-4 w-4" />
          </Button>
        </div>
      </SidebarHeader>

      <SidebarContent>
        {/* Hotel Info Card - Material Design */}
        <div className="bg-white dark:bg-gray-800/90 rounded-xl p-4 mb-4 shadow-sm border border-gray-100 dark:border-gray-700/50">
          <div className="flex items-start space-x-3 mb-3">
            <Avatar className="h-12 w-12 flex-shrink-0 shadow-md">
              <AvatarImage src={hotel.image} alt={hotel.name} />
              <AvatarFallback className="bg-gradient-to-br from-blue-600 to-blue-700 text-white font-semibold">
                <Building2 className="h-5 w-5" />
              </AvatarFallback>
            </Avatar>
            <div className="flex-1 min-w-0">
              {/* Material Design Typography - Display Small */}
              <h3 className="text-lg font-medium text-gray-900 dark:text-white leading-6 mb-1.5 tracking-tight">
                {hotel.name}
              </h3>
              {/* Material Design Typography - Body Small */}
              <div className="flex items-center space-x-1.5 mb-2">
                <div className="p-1 bg-blue-50 dark:bg-blue-900/20 rounded-full">
                  <MapPin className="h-3 w-3 text-blue-600 dark:text-blue-400 flex-shrink-0" />
                </div>
                <span className="text-sm text-gray-600 dark:text-gray-300 leading-5 font-normal">
                  {hotel.location}
                </span>
              </div>
              {/* Material Design - Manager Info */}
              <div className="flex items-center justify-between">
                <div className="flex items-center space-x-2">
                  <div className="p-1 bg-green-50 dark:bg-green-900/20 rounded-full">
                    <Users className="h-3 w-3 text-green-600 dark:text-green-400 flex-shrink-0" />
                  </div>
                  <span className="text-sm text-gray-700 dark:text-gray-300 font-medium truncate">
                    {hotel.manager.name}
                  </span>
                </div>
                {onEditHotel && (
                  <Button
                    variant="ghost"
                    size="sm"
                    onClick={onEditHotel}
                    className="h-8 w-8 p-0 text-gray-500 hover:text-gray-700 hover:bg-gray-100 dark:text-gray-400 dark:hover:text-gray-200 dark:hover:bg-gray-700/50 flex-shrink-0 rounded-full"
                  >
                    <Edit className="h-4 w-4" />
                  </Button>
                )}
              </div>
            </div>
          </div>

          {/* Material Design - Quick Stats */}
          <div className="grid grid-cols-2 gap-4 pt-3 border-t border-gray-100 dark:border-gray-700/50">
            <div className="text-center p-3 bg-gray-50 dark:bg-gray-800/50 rounded-lg">
              <div className="text-xl font-semibold text-gray-900 dark:text-white mb-1">{hotel.totalRooms}</div>
              <div className="text-xs text-gray-600 dark:text-gray-400 font-medium uppercase tracking-wide">Total Rooms</div>
            </div>
            <div className="text-center p-3 bg-gray-50 dark:bg-gray-800/50 rounded-lg">
              <div className="text-xl font-semibold text-gray-900 dark:text-white mb-1">{hotel.activeRooms}</div>
              <div className="text-xs text-gray-600 dark:text-gray-400 font-medium uppercase tracking-wide">Active Rooms</div>
            </div>
          </div>
        </div>

                  {/* Navigation - Compact */}
        <div className="space-y-4">
          <div className="px-1.5">
            <h3 className="text-xs font-medium text-muted-foreground uppercase tracking-wider">
              Navigation
            </h3>
          </div>
          <div className="space-y-2">
            {navigationItems.map((item) => {
              const Icon = item.icon
              const isActive = activeSection === item.id
              return (
                <div key={item.id}>
                  <a
                    href="#"
                    onClick={(e) => {
                      e.preventDefault()
                      onSectionChange(item.id)
                    }}
                    className={`
                      flex items-center space-x-3 px-3 py-3 rounded-md transition-all duration-200
                      ${isActive 
                        ? 'bg-blue-50 dark:bg-blue-950/50 text-blue-700 dark:text-blue-300' 
                        : 'text-gray-700 dark:text-gray-300 hover:bg-gray-50 dark:hover:bg-gray-700/50'
                      }
                    `}
                  >
                    <Icon className={`h-5 w-5 flex-shrink-0 ${isActive ? 'text-blue-600 dark:text-blue-400' : 'text-gray-500 dark:text-gray-400'}`} />
                    <div className="flex-1 min-w-0">
                      <div className={`font-medium text-base ${isActive ? 'text-blue-700 dark:text-blue-300' : 'text-gray-900 dark:text-white'}`}>
                        {item.label}
                      </div>
                    </div>
                  </a>
                </div>
              )
            })}
          </div>
        </div>

        {/* Support - Compact */}
        <div className="space-y-4 mt-8">
          <div className="px-1.5">
            <h3 className="text-xs font-medium text-muted-foreground uppercase tracking-wider">
              Support
            </h3>
          </div>
          <div className="space-y-2">
            {supportItems.map((item) => {
              const Icon = item.icon
              const isActive = activeSection === item.id
              return (
                <div key={item.id}>
                  <a
                    href="#"
                    onClick={(e) => {
                      e.preventDefault()
                      onSectionChange(item.id)
                    }}
                    className={`
                      flex items-center space-x-3 px-3 py-3 rounded-md transition-all duration-200
                      ${isActive 
                        ? 'bg-blue-50 dark:bg-blue-950/50 text-blue-700 dark:text-blue-300' 
                        : 'text-gray-700 dark:text-gray-300 hover:bg-gray-50 dark:hover:bg-gray-700/50'
                      }
                    `}
                  >
                    <Icon className={`h-5 w-5 flex-shrink-0 ${isActive ? 'text-blue-600 dark:text-blue-400' : 'text-gray-500 dark:text-gray-400'}`} />
                    <div className="flex-1 min-w-0">
                      <div className={`font-medium text-base ${isActive ? 'text-blue-700 dark:text-blue-300' : 'text-gray-900 dark:text-white'}`}>
                        {item.label}
                      </div>
                    </div>
                  </a>
                </div>
              )
            })}
          </div>
        </div>
      </SidebarContent>
    </Sidebar>
  )
} 