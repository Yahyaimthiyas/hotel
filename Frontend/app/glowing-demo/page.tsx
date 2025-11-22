import GlowingEffectDemo from "@/components/ui/glowing-effect-demo";

export default function GlowingDemoPage() {
  return (
    <div className="min-h-screen bg-background p-8">
      <div className="max-w-7xl mx-auto space-y-8">
        <div className="text-center space-y-4">
          <h1 className="text-4xl font-bold">Glowing Effect Demo</h1>
          <p className="text-muted-foreground text-lg">
            Interactive cards with glowing borders that respond to mouse movement
          </p>
        </div>
        
        <GlowingEffectDemo />
        
        <div className="text-center space-y-4">
          <h2 className="text-2xl font-semibold">Hotel Management Cards</h2>
          <p className="text-muted-foreground">
            The same glowing effect has been applied to all hotel and room cards throughout the application.
          </p>
          <div className="flex justify-center space-x-4">
            <a 
              href="/dashboard" 
              className="inline-flex items-center px-4 py-2 bg-primary text-primary-foreground rounded-md hover:bg-primary/90 transition-colors"
            >
              View Dashboard
            </a>
          </div>
        </div>
      </div>
    </div>
  );
} 