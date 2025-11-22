import * as React from "react"
import { cva, type VariantProps } from "class-variance-authority"

import { cn } from "@/lib/utils"

const badgeVariants = cva(
  "inline-flex items-center rounded-full border px-2.5 py-0.5 text-xs font-semibold transition-colors focus:outline-none focus:ring-2 focus:ring-ring focus:ring-offset-2",
  {
    variants: {
      variant: {
        default:
          "border-transparent bg-primary text-primary-foreground hover:bg-primary-hover",
        secondary:
          "border-transparent bg-secondary text-secondary-foreground hover:bg-secondary-hover",
        destructive:
          "border-transparent bg-destructive text-destructive-foreground hover:bg-destructive-hover",
        outline: "text-foreground",
        success:
          "border-transparent bg-success text-success-foreground hover:bg-success-hover",
        warning:
          "border-transparent bg-warning text-warning-foreground hover:bg-warning-hover",
        accent:
          "border-transparent bg-accent text-accent-foreground hover:bg-accent-hover",
        glass:
          "glass border-border/50 text-foreground hover:bg-accent/50",
        status: {
          active: "border-transparent bg-status-active text-white hover:bg-status-active/90",
          maintenance: "border-transparent bg-status-maintenance text-white hover:bg-status-maintenance/90",
          inactive: "border-transparent bg-status-inactive text-white hover:bg-status-inactive/90",
          pending: "border-transparent bg-status-pending text-white hover:bg-status-pending/90",
        },
      },
    },
    defaultVariants: {
      variant: "default",
    },
  }
)

export interface BadgeProps
  extends React.HTMLAttributes<HTMLDivElement>,
    VariantProps<typeof badgeVariants> {}

function Badge({ className, variant, ...props }: BadgeProps) {
  return (
    <div className={cn(badgeVariants({ variant }), className)} {...props} />
  )
}

export { Badge, badgeVariants }
