import { cn } from "@/lib/utils";
import React from "react";

export default function GridBackground() {
  return (
    <div className="absolute inset-0">
      {/* Main Grid */}
      <div
        className={cn(
          "absolute inset-0",
          "[background-size:40px_40px]",
          "[background-image:linear-gradient(to_right,#f1f5f9_0.5px,transparent_0.5px),linear-gradient(to_bottom,#f1f5f9_0.5px,transparent_0.5px)]",
          "dark:[background-image:linear-gradient(to_right,#374151_0.3px,transparent_0.3px),linear-gradient(to_bottom,#374151_0.3px,transparent_0.3px)]",
        )}
      />
      
      {/* Fade Overlay - Top */}
      <div className="absolute top-0 left-0 right-0 h-32 bg-gradient-to-b from-background via-background/80 to-transparent pointer-events-none" />
      
      {/* Fade Overlay - Bottom */}
      <div className="absolute bottom-0 left-0 right-0 h-32 bg-gradient-to-t from-background via-background/80 to-transparent pointer-events-none" />
      
      {/* Fade Overlay - Left */}
      <div className="absolute top-0 left-0 bottom-0 w-32 bg-gradient-to-r from-background via-background/80 to-transparent pointer-events-none" />
      
      {/* Fade Overlay - Right */}
      <div className="absolute top-0 right-0 bottom-0 w-32 bg-gradient-to-l from-background via-background/80 to-transparent pointer-events-none" />
      
      {/* Corner Fades for smoother transition */}
      <div className="absolute top-0 left-0 w-32 h-32 bg-gradient-to-br from-background via-background/90 to-transparent pointer-events-none" />
      <div className="absolute top-0 right-0 w-32 h-32 bg-gradient-to-bl from-background via-background/90 to-transparent pointer-events-none" />
      <div className="absolute bottom-0 left-0 w-32 h-32 bg-gradient-to-tr from-background via-background/90 to-transparent pointer-events-none" />
      <div className="absolute bottom-0 right-0 w-32 h-32 bg-gradient-to-tl from-background via-background/90 to-transparent pointer-events-none" />
    </div>
  );
} 