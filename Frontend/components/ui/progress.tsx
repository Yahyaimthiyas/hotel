"use client"

import * as React from "react"
import * as ProgressPrimitive from "@radix-ui/react-progress"

import { cn } from "@/lib/utils"

const Progress = React.forwardRef<
  React.ElementRef<typeof ProgressPrimitive.Root>,
  React.ComponentPropsWithoutRef<typeof ProgressPrimitive.Root> & {
    variant?: "default" | "primary" | "secondary" | "accent" | "success" | "warning" | "destructive" | "gradient"
  }
>(({ className, value, variant = "default", ...props }, ref) => {
  const getVariantClasses = () => {
    switch (variant) {
      case "primary":
        return "bg-primary"
      case "secondary":
        return "bg-secondary"
      case "accent":
        return "bg-accent"
      case "success":
        return "bg-success"
      case "warning":
        return "bg-warning"
      case "destructive":
        return "bg-destructive"
      case "gradient":
        return "gradient-primary"
      default:
        return "bg-primary"
    }
  }

  return (
    <ProgressPrimitive.Root
      ref={ref}
      className={cn(
        "relative h-2 w-full overflow-hidden rounded-full bg-muted",
        className
      )}
      {...props}
    >
      <ProgressPrimitive.Indicator
        className={cn(
          "h-full w-full flex-1 transition-all duration-300 ease-in-out",
          getVariantClasses()
        )}
        style={{ transform: `translateX(-${100 - (value || 0)}%)` }}
      />
    </ProgressPrimitive.Root>
  )
})
Progress.displayName = ProgressPrimitive.Root.displayName

export { Progress }
