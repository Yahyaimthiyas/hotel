import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { AlertTriangle } from "lucide-react"
import Link from "next/link"


export default function UnauthorizedPage() {
  return (
    <div className="min-h-screen flex items-center justify-center bg-background p-4">
      <Card className="w-full max-w-md">
        <CardHeader className="text-center">
          <div className="mx-auto mb-4 flex h-12 w-12 items-center justify-center rounded-full bg-red-100 dark:bg-red-900">
            <AlertTriangle className="h-6 w-6 text-red-600 dark:text-red-400" />
          </div>
          <CardTitle className="text-2xl">Access Denied</CardTitle>
          <CardDescription>You don't have permission to access this resource.</CardDescription>
        </CardHeader>
        <CardContent className="text-center space-y-4">
          <p className="text-sm text-muted-foreground">
            Please contact your administrator if you believe this is an error.
          </p>
          <div className="space-y-2">
            <Button asChild className="w-full">
              <Link href="/dashboard">Return to Dashboard</Link>
            </Button>
            <Button asChild variant="outline" className="w-full bg-transparent">
              <Link href="/login">Sign In Again</Link>
            </Button>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}
