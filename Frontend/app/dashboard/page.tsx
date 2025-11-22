"use client"

/**
 * @file page.tsx
 * @brief Entry point for the authenticated dashboard, routing between
 *        admin, staff, and guest dashboards and rendering a floating
 *        navbar with user controls.
 */

import { useState } from "react"
import { useAuth } from "@/lib/auth"
import { ProtectedRoute } from "@/components/auth/protected-route"
import { AdminDashboard } from "@/components/dashboards/admin-dashboard"
import { StaffDashboard } from "@/components/dashboards/staff-dashboard"
import { GuestDashboard } from "@/components/dashboards/guest-dashboard"
import { ThemeToggle } from "@/components/theme-toggle"
import { Button } from "@/components/ui/button"
import { Avatar, AvatarFallback, AvatarImage } from "@/components/ui/avatar"
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from "@/components/ui/dropdown-menu"
import { LogOut, Settings, User } from "lucide-react"
import { useScrollDirection } from "@/lib/hooks/useScrollDirection"
import { useDeviceDetection } from "@/hooks/use-mobile"

/**
 * Main dashboard page wrapped in ProtectedRoute, selects the correct
 * role-specific dashboard and renders a responsive floating navbar.
 */
export default function DashboardPage() {
  const { user, logout } = useAuth()
  const { scrollDirection, isAtTop } = useScrollDirection()
  const [isHotelView, setIsHotelView] = useState(false)
  const { deviceType, isMobile, isTablet, isDesktop, orientation, shouldShowMobileLayout } = useDeviceDetection()

  /** Trigger logout via auth context. */
  const handleLogout = () => {
    logout()
  }

  /**
   * Decide which dashboard component to render based on user role.
   */
  const getDashboardComponent = () => {
    if (!user) return null

    switch (user.role) {
      case "administrator":
        return <AdminDashboard onHotelViewChange={setIsHotelView} />
      case "staff":
        return <StaffDashboard />
      case "guest":
        return <GuestDashboard />
      default:
        return <div>Invalid role</div>
    }
  }

  return (
    <ProtectedRoute>
      <div className="min-h-screen bg-background relative safe-area-inset-top safe-area-inset-bottom">

        {/* Floating Navbar - Enhanced for all devices */}
        {!isHotelView && (
          <div 
            className={`
              fixed top-2 xs:top-3 sm:top-4 md:top-6 left-1/2 transform -translate-x-1/2 z-50 transition-all duration-500 ease-out
              ${scrollDirection === 'down' && !isAtTop ? 'animate-[navbar-slide-down_0.5s_ease-out_forwards]' : ''}
              ${scrollDirection === 'up' || isAtTop ? 'animate-[navbar-slide-up_0.5s_ease-out_forwards]' : ''}
              ${isAtTop ? 'animate-[navbar-float_6s_ease-in-out_infinite]' : ''}
            `}
          >
          <div className="glass-pill rounded-full px-3 xs:px-4 sm:px-6 md:px-8 py-2 xs:py-3 md:py-3 border border-white/20 dark:border-white/10 touch-target" style={{ boxShadow: 'none' }}>
            <div className="flex items-center space-x-2 xs:space-x-3 sm:space-x-4 md:space-x-6">
              {/* Logo Section - Responsive sizing */}
              <div className="flex items-center space-x-1.5 xs:space-x-2 sm:space-x-3">
                <div className="w-6 h-6 xs:w-7 xs:h-7 sm:w-8 sm:h-8 md:w-9 md:h-9 rounded-lg gradient-primary flex items-center justify-center">
                  <span className="text-white font-bold text-xs xs:text-sm sm:text-base">R</span>
                </div>
                <div className="hidden sm:block">
                  <h1 className="text-sm xs:text-base sm:text-lg md:text-xl font-bold text-gradient-primary">RFID Manager</h1>
                  <p className="text-xs xs:text-sm text-muted-foreground">Hotel Key Card System</p>
                </div>
              </div>

              {/* Divider - Hidden on mobile */}
              <div className="hidden sm:block w-px h-6 bg-border/50"></div>

              {/* User Info - Responsive layout */}
              <div className="flex items-center space-x-1.5 xs:space-x-2 sm:space-x-3 md:space-x-4">
                <div className="hidden lg:block text-right">
                  <p className="text-sm font-medium text-foreground">{user?.name}</p>
                  <p className="text-xs text-muted-foreground capitalize">{user?.role}</p>
                </div>
                
                {/* Theme Toggle - Enhanced touch target */}
                <div className="touch-target">
                  <ThemeToggle />
                </div>

                {/* User Menu - Enhanced for touch devices */}
                <DropdownMenu>
                  <DropdownMenuTrigger asChild>
                    <Button variant="ghost" className="relative h-6 w-6 xs:h-7 xs:w-7 sm:h-8 sm:w-8 md:h-9 md:w-9 rounded-full hover:bg-white/10 touch-target">
                      <Avatar className="h-6 w-6 xs:h-7 xs:w-7 sm:h-8 sm:w-8 md:h-9 md:w-9">
                        <AvatarImage src={user?.avatar || "/placeholder.svg"} alt={user?.name} />
                        <AvatarFallback className="text-xs xs:text-sm">
                          {user?.name
                            ?.split(" ")
                            .map((n) => n[0])
                            .join("")}
                        </AvatarFallback>
                      </Avatar>
                    </Button>
                  </DropdownMenuTrigger>
                  <DropdownMenuContent className="w-56 glass-strong border-border/50" align="end" forceMount>
                    <DropdownMenuLabel className="font-normal">
                      <div className="flex flex-col space-y-1">
                        <p className="text-sm font-medium leading-none">{user?.name}</p>
                        <p className="text-xs leading-none text-muted-foreground">{user?.email}</p>
                        <p className="text-xs leading-none text-muted-foreground capitalize">
                          {user?.role} {user?.department && `• ${user.department}`}
                        </p>
                      </div>
                    </DropdownMenuLabel>
                    <DropdownMenuSeparator />
                    <DropdownMenuItem className="hover:bg-accent/50 touch-target">
                      <User className="mr-2 h-4 w-4" />
                      <span>Profile</span>
                    </DropdownMenuItem>
                    <DropdownMenuItem className="hover:bg-accent/50 touch-target">
                      <Settings className="mr-2 h-4 w-4" />
                      <span>Settings</span>
                    </DropdownMenuItem>
                    <DropdownMenuSeparator />
                    <DropdownMenuItem onClick={handleLogout} className="hover:bg-destructive/10 text-destructive touch-target">
                      <LogOut className="mr-2 h-4 w-4" />
                      <span>Log out</span>
                    </DropdownMenuItem>
                  </DropdownMenuContent>
                </DropdownMenu>
              </div>
            </div>
          </div>
        </div>
        )}

        {/* Main Content - Enhanced responsive padding and layout */}
        <div className={`
          ${isHotelView ? "relative z-10 h-screen" : `
            w-full 
            px-3 xs:px-4 sm:px-6 md:px-8 lg:px-12 xl:px-16 2xl:px-20 
            pt-20 xs:pt-24 sm:pt-28 md:pt-32 lg:pt-36 
            pb-4 xs:pb-6 sm:pb-8 md:pb-12 
            relative z-10
          `}
        `}>
          {getDashboardComponent()}
        </div>
      </div>
    </ProtectedRoute>
  )
}
