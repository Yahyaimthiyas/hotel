"use client"

import * as React from "react"
import { cn } from "@/lib/utils"
import { Button } from "@/components/ui/button"
import { ScrollArea } from "@/components/ui/scroll-area"
import { Sheet, SheetContent, SheetTrigger, SheetTitle } from "@/components/ui/sheet"
import { Menu, X } from "lucide-react"
import { useDeviceDetection } from "@/hooks/use-mobile"

interface SidebarProps extends React.HTMLAttributes<HTMLDivElement> {
  children: React.ReactNode
  isOpen?: boolean
  onOpenChange?: (open: boolean) => void
  showMobileToggle?: boolean
}

export function Sidebar({
  className,
  children,
  isOpen = false,
  onOpenChange,
  showMobileToggle = true,
  ...props
}: SidebarProps) {
  const { deviceType, isMobile, isTablet, isDesktop, orientation } = useDeviceDetection();

  return (
    <>
      {/* Mobile Sidebar - Enhanced for all mobile devices */}
      {showMobileToggle && (
        <Sheet open={isOpen} onOpenChange={onOpenChange}>
          <SheetContent 
            side="left" 
            className={`
              pl-0 pr-0 pt-0 
              w-[280px] xs:w-[320px] sm:w-[360px] md:w-[400px]
              bg-white dark:bg-gray-900 h-full
              safe-area-inset-left safe-area-inset-right
            `} 
 
            hideCloseButton
          >
            <SheetTitle className="sr-only">Hotel Management Sidebar</SheetTitle>
            <div className="h-full flex flex-col">
              <ScrollArea className="flex-1 py-2 xs:py-3 sm:py-4 mobile-scroll">
                <div className="space-y-2 xs:space-y-3 sm:space-y-4 px-2 xs:px-3 sm:px-4">
                  {children}
                </div>
              </ScrollArea>
            </div>
          </SheetContent>
        </Sheet>
      )}

      {/* Desktop Sidebar - Enhanced responsive sizing */}
      <div
        className={cn(
          `
            hidden lg:block 
            w-[280px] xl:w-[320px] 2xl:w-[360px] 
            bg-white dark:bg-gray-900 
            border-r border-gray-200 dark:border-gray-800 
            flex-shrink-0
            safe-area-inset-left
          `,
          className
        )}

        {...props}
      >
        <div className="h-full flex flex-col">
          <ScrollArea className="flex-1 py-2 xs:py-3 sm:py-4">
            <div className="space-y-2 xs:space-y-3 sm:space-y-4 px-2 xs:px-3 sm:px-4">
              {children}
            </div>
          </ScrollArea>
        </div>
      </div>
    </>
  )
}

interface SidebarHeaderProps extends React.HTMLAttributes<HTMLDivElement> {
  children: React.ReactNode
}

export function SidebarHeader({ className, children, ...props }: SidebarHeaderProps) {
  return (
    <div
      className={cn("flex h-[50px] xs:h-[55px] sm:h-[60px] items-center px-3 xs:px-4", className)}
      {...props}
    >
      {children}
    </div>
  )
}

interface SidebarContentProps extends React.HTMLAttributes<HTMLDivElement> {
  children: React.ReactNode
}

export function SidebarContent({ className, children, ...props }: SidebarContentProps) {
  return (
    <div className={cn("flex-1 overflow-auto mobile-scroll", className)} {...props}>
      {children}
    </div>
  )
}

interface SidebarFooterProps extends React.HTMLAttributes<HTMLDivElement> {
  children: React.ReactNode
}

export function SidebarFooter({ className, children, ...props }: SidebarFooterProps) {
  return (
    <div
      className={cn("flex h-[50px] xs:h-[55px] sm:h-[60px] items-center px-3 xs:px-4", className)}
      {...props}
    >
      {children}
    </div>
  )
}

interface SidebarNavProps extends React.HTMLAttributes<HTMLElement> {
  children: React.ReactNode
}

export function SidebarNav({ className, children, ...props }: SidebarNavProps) {
  return (
    <nav className={cn("flex flex-col space-y-1", className)} {...props}>
      {children}
    </nav>
  )
}

interface SidebarNavItemProps extends React.HTMLAttributes<HTMLLIElement> {
  children: React.ReactNode
  isActive?: boolean
}

export function SidebarNavItem({ className, children, isActive, ...props }: SidebarNavItemProps) {
  return (
    <li
      className={cn(
        "relative",
        className
      )}
      {...props}
    >
      {children}
    </li>
  )
}

interface SidebarNavLinkProps extends React.HTMLAttributes<HTMLAnchorElement> {
  children: React.ReactNode
  isActive?: boolean
  href?: string
}

export function SidebarNavLink({ className, children, isActive, href, ...props }: SidebarNavLinkProps) {
  return (
    <a
      href={href}
      className={cn(
        "group flex items-center rounded-md px-3 py-2 text-sm font-medium touch-target",
        className
      )}
      {...props}
    >
      {children}
    </a>
  )
}
