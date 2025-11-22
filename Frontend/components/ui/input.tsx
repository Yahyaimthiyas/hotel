import * as React from "react"

import { cn } from "@/lib/utils"

export interface InputProps
  extends React.InputHTMLAttributes<HTMLInputElement> {
  variant?: "default" | "glass" | "glassStrong"
}

const Input = React.forwardRef<HTMLInputElement, InputProps>(
  ({ className, type, variant = "default", ...props }, ref) => {
    const getVariantClasses = () => {
      switch (variant) {
        case "glass":
          return "glass border-border/50 focus:border-primary focus:ring-2 focus:ring-primary/20"
        case "glassStrong":
          return "glass-strong border-border/50 focus:border-primary focus:ring-2 focus:ring-primary/20"
        default:
          return "border border-input bg-background ring-offset-background focus-visible:ring-2 focus-visible:ring-ring focus-visible:ring-offset-2"
      }
    }

    // Remove any browser extension attributes that cause hydration issues
    const cleanProps = { ...props }
    if (typeof window !== 'undefined') {
      // Remove attributes that might be added by browser extensions
      delete (cleanProps as any).fdprocessedid
    }

    return (
      <input
        type={type}
        className={cn(
          "flex h-10 w-full rounded-md px-3 py-2 text-sm transition-all duration-200 disabled:cursor-not-allowed disabled:opacity-50",
          getVariantClasses(),
          className
        )}
        ref={ref}
        {...cleanProps}
      />
    )
  }
)
Input.displayName = "Input"

export { Input }