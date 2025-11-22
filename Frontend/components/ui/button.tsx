import * as React from "react"
import { Slot } from "@radix-ui/react-slot"
import { cva, type VariantProps } from "class-variance-authority"

import { cn } from "@/lib/utils"

const buttonVariants = cva(
  "inline-flex items-center justify-center whitespace-nowrap rounded-md text-sm font-medium ring-offset-background transition-all duration-200 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring focus-visible:ring-offset-2 disabled:pointer-events-none disabled:opacity-50",
  {
    variants: {
      variant: {
        default: "bg-primary text-primary-foreground hover:bg-primary-hover shadow-md hover:shadow-lg",
        destructive:
          "bg-destructive text-destructive-foreground hover:bg-destructive-hover shadow-md hover:shadow-lg",
        outline:
          "border border-input bg-background hover:bg-accent hover:text-accent-foreground",
        secondary:
          "bg-secondary text-secondary-foreground hover:bg-secondary-hover shadow-md hover:shadow-lg",
        ghost: "hover:bg-accent hover:text-accent-foreground",
        link: "text-primary underline-offset-4 hover:underline",
        gradient: "gradient-primary text-primary-foreground shadow-lg hover:shadow-xl hover:scale-[1.02]",
        gradientSecondary: "gradient-secondary text-secondary-foreground shadow-lg hover:shadow-xl hover:scale-[1.02]",
        gradientAccent: "gradient-accent text-accent-foreground shadow-lg hover:shadow-xl hover:scale-[1.02]",
        gradientSuccess: "gradient-success text-success-foreground shadow-lg hover:shadow-xl hover:scale-[1.02]",
        gradientWarning: "gradient-warning text-warning-foreground shadow-lg hover:shadow-xl hover:scale-[1.02]",
        gradientDestructive: "gradient-destructive text-destructive-foreground shadow-lg hover:shadow-xl hover:scale-[1.02]",
        glass: "glass border-border/50 hover:bg-accent/50 hover:border-accent/50",
        glassStrong: "glass-strong border-border/50 hover:bg-accent/50 hover:border-accent/50",
      },
      size: {
        default: "h-10 px-4 py-2",
        sm: "h-9 rounded-md px-3",
        lg: "h-11 rounded-md px-8",
        xl: "h-14 rounded-lg px-10 text-base",
        icon: "h-10 w-10",
      },
    },
    defaultVariants: {
      variant: "default",
      size: "default",
    },
  }
)

export interface ButtonProps
  extends React.ButtonHTMLAttributes<HTMLButtonElement>,
    VariantProps<typeof buttonVariants> {
  asChild?: boolean
}

const Button = React.forwardRef<HTMLButtonElement, ButtonProps>(
  ({ className, variant, size, asChild = false, ...props }, ref) => {
    const Comp = asChild ? Slot : "button"
    return (
      <Comp
        className={cn(buttonVariants({ variant, size, className }))}
        ref={ref}
        {...props}
      />
    )
  }
)
Button.displayName = "Button"

export { Button, buttonVariants }
