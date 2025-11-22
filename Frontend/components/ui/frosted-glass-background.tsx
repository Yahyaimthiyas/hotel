"use client"

import React from "react"

interface FrostedGlassBackgroundProps {
  children: React.ReactNode
  className?: string
}

export function FrostedGlassBackground({ children, className = "" }: FrostedGlassBackgroundProps) {
  return (
    <div className={`min-h-screen w-full relative overflow-hidden ${className}`}>
      {/* Actual Figma wallpaper background */}
      <div 
        className="absolute inset-0 bg-cover bg-center bg-no-repeat"
        style={{
          backgroundImage: 'url(/07.png)'
        }}
      />
      
      {/* Subtle overlay for better text readability */}
      <div className="absolute inset-0 bg-black/10" />
      
      {/* Main content container */}
      <div className="relative z-10 min-h-screen flex items-center justify-center px-4 py-4 sm:px-6 sm:py-6 lg:px-8 lg:py-8">
        {children}
      </div>
    </div>
  )
} 