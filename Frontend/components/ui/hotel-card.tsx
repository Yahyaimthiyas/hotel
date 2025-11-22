"use client";

import { Building2, MapPin, Users, CreditCard } from "lucide-react";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { GlowingEffect } from "@/components/ui/glowing-effect";
import { ModernCard } from "@/components/ui/card";
import { useDeviceDetection } from "@/hooks/use-mobile";

interface HotelCardProps {
  hotel: {
    id: string;
    name: string;
    location: string;
    manager: {
      name: string;
      phone: string;
      email: string;
      status: string;
    };
    totalRooms?: number;
    activeRooms?: number;
    totalCards?: number;
    status: string;
    occupancy?: number;
    lastActivity: string;
    image: string;
  };
  isSelected?: boolean;
  onClick?: () => void;
}

export function HotelCard({ hotel, isSelected = false, onClick }: HotelCardProps) {
  const { deviceType, isMobile, isTablet, isDesktop, orientation } = useDeviceDetection();

  return (
    <div 
      className={`
        relative h-full 
        min-h-[180px] xs:min-h-[200px] sm:min-h-[220px] md:min-h-[240px] lg:min-h-[260px] 
        rounded-lg xs:rounded-xl sm:rounded-2xl 
        border p-2 xs:p-3 sm:p-4 
        cursor-pointer 
        transition-all duration-300 
        hover:scale-[1.02] hover:shadow-lg 
        touch-target
        ${isSelected ? "ring-2 ring-primary shadow-xl" : "hover:border-primary/50"}
      `}
      onClick={onClick}
    >
      <GlowingEffect
        spread={40}
        glow={true}
        disabled={false}
        proximity={64}
        inactiveZone={0.01}
      />
      <ModernCard variant="glassStrong" className="relative flex h-full flex-col justify-between gap-2 xs:gap-3 sm:gap-4 overflow-hidden p-2 xs:p-3 sm:p-4">
        <div className="relative flex flex-1 flex-col justify-between gap-2 xs:gap-3">
          {/* Header */}
          <div className="space-y-1.5 xs:space-y-2 sm:space-y-3">
            <div className="w-fit rounded-lg gradient-primary p-1 xs:p-1.5 sm:p-2 shadow-md">
              <Building2 className="h-2.5 xs:h-3 sm:h-4 w-2.5 xs:w-3 sm:w-4 text-primary-foreground" />
            </div>
            <div className="space-y-1 xs:space-y-1.5 sm:space-y-2">
              <h3 className="font-sans text-xs xs:text-sm sm:text-base md:text-lg font-bold text-balance text-foreground tracking-tight leading-tight">
                {hotel.name}
              </h3>
              <div className="flex items-center space-x-1 text-xs xs:text-sm text-muted-foreground">
                <MapPin className="h-2.5 xs:h-3 sm:h-3.5 w-2.5 xs:w-3 sm:w-3.5 text-accent flex-shrink-0" />
                <span className="truncate">{hotel.location}</span>
              </div>
              <div className="flex items-center space-x-1 text-xs xs:text-sm text-muted-foreground">
                <Users className="h-2.5 xs:h-3 sm:h-3.5 w-2.5 xs:w-3 sm:w-3.5 text-primary flex-shrink-0" />
                <span className="truncate">Manager: {hotel.manager.name}</span>
              </div>
            </div>
          </div>

          {/* Metrics */}
          <div className="space-y-1.5 xs:space-y-2 sm:space-y-3">
            <div className="grid grid-cols-2 gap-1.5 xs:gap-2 sm:gap-3">
              <div className="text-center p-1.5 xs:p-2 sm:p-3 rounded-lg gradient-card-enhanced border border-border/50">
                <div className="flex items-center justify-center mb-1">
                  <Users className="h-2.5 xs:h-3 sm:h-3.5 w-2.5 xs:w-3 sm:w-3.5 text-success" />
                </div>
                <p className="text-xs xs:text-sm sm:text-base font-bold text-foreground">{hotel.totalRooms || 0}</p>
                <p className="text-xs text-muted-foreground font-medium">Total Rooms</p>
              </div>
              <div className="text-center p-1.5 xs:p-2 sm:p-3 rounded-lg gradient-card-enhanced border border-border/50">
                <div className="flex items-center justify-center mb-1">
                  <CreditCard className="h-2.5 xs:h-3 sm:h-3.5 w-2.5 xs:w-3 sm:w-3.5 text-primary" />
                </div>
                <p className="text-xs xs:text-sm sm:text-base font-bold text-foreground">{hotel.activeRooms || 0}</p>
                <p className="text-xs text-muted-foreground font-medium">Active Rooms</p>
              </div>
            </div>

            {/* Last Activity */}
            <div className="flex items-center justify-between text-xs xs:text-sm text-muted-foreground bg-muted/30 rounded-lg p-1 xs:p-1.5 sm:p-2">
              <span className="font-medium">Last activity:</span>
              <span className="truncate">{hotel.lastActivity}</span>
            </div>

            {/* Action Button */}
            <Button 
              variant="gradient"
              className="w-full h-6 xs:h-7 sm:h-8 md:h-9 touch-target" 
              size="sm"
              onClick={(e) => {
                e.stopPropagation();
                onClick?.();
              }}
            >
              <Building2 className="h-2.5 xs:h-3 sm:h-3.5 w-2.5 xs:w-3 sm:w-3.5 mr-1" />
              <span className="text-xs xs:text-sm">Manage Hotel</span>
            </Button>
          </div>
        </div>
      </ModernCard>
    </div>
  );
} 