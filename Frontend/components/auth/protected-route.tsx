"use client"

import type React from "react"

import { useAuth } from "@/lib/auth"
import { useRouter } from "next/navigation"
import { useEffect } from "react"
import { Loader2 } from "lucide-react"

interface ProtectedRouteProps {
  children: React.ReactNode
  requiredRole?: "administrator" | "staff" | "guest"
  requiredPermissions?: string[]
}

export function ProtectedRoute({ children, requiredRole, requiredPermissions }: ProtectedRouteProps) {
  const { user, loading } = useAuth()
  const router = useRouter()

  useEffect(() => {
    if (!loading && !user) {
      router.push("/login")
      return
    }

    if (user && requiredRole && user.role !== requiredRole) {
      router.push("/unauthorized")
      return
    }

    if (user && requiredPermissions) {
      const hasPermissions = requiredPermissions.every((permission) => user.permissions.includes(permission))
      if (!hasPermissions) {
        router.push("/unauthorized")
        return
      }
    }
  }, [user, loading, router, requiredRole, requiredPermissions])

  if (loading) {
    return (
      <div className="min-h-screen flex items-center justify-center">
        <div className="text-center">
          <Loader2 className="h-8 w-8 animate-spin mx-auto mb-4" />
          <p className="text-muted-foreground">Loading...</p>
        </div>
      </div>
    )
  }

  if (!user) {
    return null
  }

  if (requiredRole && user.role !== requiredRole) {
    return null
  }

  if (requiredPermissions) {
    const hasPermissions = requiredPermissions.every((permission) => user.permissions.includes(permission))
    if (!hasPermissions) {
      return null
    }
  }

  return <>{children}</>
}


