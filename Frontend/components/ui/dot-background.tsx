import { cn } from "@/lib/utils";
import React from "react";

interface DotBackgroundProps {
  children?: React.ReactNode;
  className?: string;
  containerClassName?: string;
  showRadialGradient?: boolean;
  dotColor?: string;
  dotColorDark?: string;
  dotSize?: string;
  dotSpacing?: string;
}

export function DotBackground({
  children,
  className,
  containerClassName,
  showRadialGradient = true,
  dotColor = "#d4d4d4",
  dotColorDark = "#404040",
  dotSize = "1px",
  dotSpacing = "20px"
}: DotBackgroundProps) {
  return (
    <div className={cn("relative flex min-h-screen w-full items-center justify-center bg-white dark:bg-background", containerClassName)}>
      <div
        className={cn(
          "absolute inset-0",
          `[background-size:${dotSpacing}_${dotSpacing}]`,
          `[background-image:radial-gradient(${dotColor}_${dotSize},transparent_${dotSize})]`,
          `dark:[background-image:radial-gradient(${dotColorDark}_${dotSize},transparent_${dotSize})]`,
        )}
      />
      {/* Radial gradient for the container to give a faded look */}
      {showRadialGradient && (
        <div className="pointer-events-none absolute inset-0 flex items-center justify-center bg-white [mask-image:radial-gradient(ellipse_at_center,transparent_20%,black)] dark:bg-background"></div>
      )}
      <div className={cn("relative z-20", className)}>
        {children}
      </div>
    </div>
  );
} 