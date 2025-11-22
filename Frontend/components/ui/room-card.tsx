"use client";

import { 
  Bed, 
  User, 
  Shield, 
  Eye, 
  Settings 
} from "lucide-react";
import { Badge } from "@/components/ui/badge";
import { Button } from "@/components/ui/button";
import { Avatar, AvatarImage, AvatarFallback } from "@/components/ui/avatar";
import { GlowingEffect } from "@/components/ui/glowing-effect";

interface RoomCardProps {
  room: {
    id: string;
    number: string;
    type: string;
    floor: string;
    occupancy: string;
    lastActivity: string;
    occupant?: {
      name: string;
      type: string;
      avatar: string;
      checkIn?: string;
      checkOut?: string;
    };
  };
  onClick?: () => void;
}

export function RoomCard({ room, onClick }: RoomCardProps) {
  const getOccupancyColor = (occupancy: string) => {
    switch (occupancy) {
      case "vacant":
        return "bg-green-500";
      case "occupied":
        return "bg-red-500";
      case "maintenance":
      case "cleaning":
        return "bg-yellow-500";
      case "dirty":
        return "bg-amber-700";
      default:
        return "bg-gray-500";
    }
  };

  const getOccupancyText = (occupancy: string) => {
    switch (occupancy) {
      case "vacant":
        return "Vacant";
      case "occupied":
        return "Occupied";
      case "maintenance":
        return "Maintenance";
      case "cleaning":
        return "Cleaning";
      case "dirty":
        return "Dirty";
      default:
        return "Unknown";
    }
  };



  const getOccupantTypeIcon = (type: string) => {
    switch (type) {
      case "guest":
        return <User className="h-4 w-4 text-blue-500" />;
      case "manager":
        return <Shield className="h-4 w-4 text-purple-500" />;
      default:
        return <User className="h-4 w-4" />;
    }
  };

  return (
    <div className="relative h-full rounded-2xl border p-2 md:rounded-3xl md:p-3 hover:shadow-lg transition-shadow">
      <GlowingEffect
        spread={40}
        glow={true}
        disabled={false}
        proximity={64}
        inactiveZone={0.01}
      />
      <div className="border-0.75 relative flex h-full flex-col justify-between gap-4 overflow-hidden rounded-xl p-4 md:p-4 dark:shadow-[0px_0px_27px_0px_#2D2D2D]">
        <div className="relative flex flex-1 flex-col justify-between gap-2">
                      {/* Header */}
            <div className="space-y-2">
              <div className="flex items-center justify-between">
                <div className="w-fit rounded-lg border border-gray-600 p-1.5">
                  <Bed className="h-3 w-3 text-black dark:text-neutral-400" />
                </div>
                <div className="flex items-center space-x-1.5 sm:space-x-2">
                  <div className={`w-2.5 h-2.5 sm:w-3 sm:h-3 rounded-full ${getOccupancyColor(room.occupancy)} flex-shrink-0`}></div>
                  <Badge variant="outline" className="text-xs px-1.5 py-0.5 sm:px-2 sm:py-1 flex-shrink-0">
                    {getOccupancyText(room.occupancy)}
                  </Badge>
                </div>
              </div>
              <div className="space-y-1">
                <h3 className="-tracking-4 pt-0.5 font-sans text-lg font-semibold text-balance text-black dark:text-white">
                  Room {room.number}
                </h3>
                <div className="flex items-center space-x-2 text-xs text-black dark:text-neutral-400">
                  <span>{room.type}</span>
                  <span>•</span>
                  <span>Floor {room.floor}</span>
                </div>
              </div>
            </div>

          {/* Content */}
          <div className="space-y-2">
            {/* Occupant Info */}
            {room.occupant ? (
              <div className="flex items-center space-x-2 p-3 bg-gray-100 dark:bg-gray-800 rounded-lg border border-gray-200 dark:border-gray-700">
                <Avatar className="h-7 w-7">
                  <AvatarImage src={room.occupant.avatar} alt={room.occupant.name} />
                  <AvatarFallback className="text-xs bg-blue-100 dark:bg-blue-900 text-blue-700 dark:text-blue-300">
                    {room.occupant.name.split(" ").map((n) => n[0]).join("")}
                  </AvatarFallback>
                </Avatar>
                <div className="flex-1 min-w-0">
                  <p className="text-sm font-semibold text-gray-900 dark:text-gray-100 truncate">{room.occupant.name}</p>
                  <div className="flex items-center space-x-1 text-xs text-gray-600 dark:text-gray-400">
                    {getOccupantTypeIcon(room.occupant.type)}
                    <span className="capitalize">{room.occupant.type}</span>
                  </div>
                </div>
              </div>
            ) : (
              <div className="p-3 bg-gray-50 dark:bg-gray-900 rounded-lg text-center border border-gray-200 dark:border-gray-700">
                <p className="text-xs text-gray-500 dark:text-gray-400 font-medium">Vacant</p>
              </div>
            )}

            {/* Last Activity */}
            <div className="flex items-center justify-between text-xs text-muted-foreground">
              <span>Last activity:</span>
              <span>{room.lastActivity}</span>
            </div>

            {/* Actions */}
            <div className="flex items-center space-x-2">
              <Button size="sm" className="flex-1" onClick={onClick}>
                <Eye className="h-3 w-3 mr-1" />
                Details
              </Button>
              <Button size="sm" variant="outline">
                <Settings className="h-3 w-3" />
              </Button>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
} 