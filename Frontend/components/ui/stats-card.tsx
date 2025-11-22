"use client";

import { LucideIcon } from "lucide-react";
import { ModernCard, CardContent } from "@/components/ui/card";
import { cn } from "@/lib/utils";

interface StatsCardProps {
  title: string;
  value: string | number;
  description?: string;
  icon: LucideIcon;
  trend?: {
    value: number;
    isPositive: boolean;
  };
  variant?: "default" | "primary" | "secondary" | "accent" | "success" | "warning" | "destructive";
  className?: string;
}

export function StatsCard({ 
  title, 
  value, 
  description, 
  icon: Icon, 
  trend, 
  variant = "default",
  className 
}: StatsCardProps) {
  const getVariantStyles = () => {
    switch (variant) {
      case "primary":
        return {
          icon: "text-primary",
          gradient: "gradient-primary",
          bg: "bg-primary-light/20"
        };
      case "secondary":
        return {
          icon: "text-secondary",
          gradient: "gradient-secondary",
          bg: "bg-secondary-light/20"
        };
      case "accent":
        return {
          icon: "text-accent",
          gradient: "gradient-accent",
          bg: "bg-accent-light/20"
        };
      case "success":
        return {
          icon: "text-success",
          gradient: "gradient-success",
          bg: "bg-success-light/20"
        };
      case "warning":
        return {
          icon: "text-warning",
          gradient: "gradient-warning",
          bg: "bg-warning-light/20"
        };
      case "destructive":
        return {
          icon: "text-destructive",
          gradient: "gradient-destructive",
          bg: "bg-destructive-light/20"
        };
      default:
        return {
          icon: "text-muted-foreground",
          gradient: "gradient-card",
          bg: "bg-muted/30"
        };
    }
  };

  const styles = getVariantStyles();

  return (
    <ModernCard variant="glassStrong" className={cn("hover:scale-[1.02]", className)}>
      <CardContent className="p-4">
        <div className="flex items-center justify-between">
          <div className="space-y-1">
            <p className="text-xs font-medium text-muted-foreground">{title}</p>
            <div className="flex items-baseline space-x-2">
              <p className="text-2xl font-bold text-foreground">{value}</p>
              {trend && (
                <span className={cn(
                  "text-xs font-medium",
                  trend.isPositive ? "text-success" : "text-destructive"
                )}>
                  {trend.isPositive ? "+" : ""}{trend.value}%
                </span>
              )}
            </div>
            {description && (
              <p className="text-xs text-muted-foreground">{description}</p>
            )}
          </div>
          <div className={cn(
            "p-1.5 rounded-lg shadow-md",
            styles.bg
          )}>
            <Icon className={cn("h-5 w-5", styles.icon)} />
          </div>
        </div>
      </CardContent>
    </ModernCard>
  );
} 