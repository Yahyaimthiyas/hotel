"use client"

import { useState } from "react"
import { useAuth } from "@/lib/auth"
import { useRouter } from "next/navigation"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Building2, Eye, EyeOff, Lock, Mail, Smartphone, Tablet, Monitor } from "lucide-react"
import { useDeviceDetection } from "@/hooks/use-mobile"

export function LoginForm() {
  const [email, setEmail] = useState("")
  const [password, setPassword] = useState("")
  const [showPassword, setShowPassword] = useState(false)
  const [isLoading, setIsLoading] = useState(false)
  const [error, setError] = useState("")
  const { login } = useAuth()
  const router = useRouter()
  const { deviceType, isMobile, isTablet, isDesktop, orientation } = useDeviceDetection()

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault()
    setIsLoading(true)
    setError("")

    try {
      const result = await login(email, password)
      
      if (result.success) {
        // Redirect to dashboard on successful login
        router.push("/dashboard")
      } else {
        setError(result.error || "Login failed")
      }
    } catch (err) {
      setError("An error occurred during login")
    } finally {
      setIsLoading(false)
    }
  }

  const getDeviceIcon = () => {
    if (isMobile) return <Smartphone className="h-4 w-4" />
    if (isTablet) return <Tablet className="h-4 w-4" />
    return <Monitor className="h-4 w-4" />
  }

  return (
    <div className="flex items-center justify-center min-h-screen p-3 xs:p-4 sm:p-6 md:p-8 safe-area-inset-top safe-area-inset-bottom">
      <div className="w-full max-w-[280px] xs:max-w-[320px] sm:max-w-[360px] md:max-w-[400px] lg:max-w-[440px] xl:max-w-[480px]">
        
        {/* Main Form Card */}
        <div className="frosted-glass dark:frosted-glass-dark shadow-2xl rounded-2xl xs:rounded-3xl p-4 xs:p-5 sm:p-6 md:p-8 relative overflow-hidden">
          
          {/* Background Effects */}
          <div className="absolute inset-0 bg-gradient-to-br from-white/5 to-transparent rounded-2xl xs:rounded-3xl" />
          <div className="absolute inset-0 bg-gradient-to-tl from-white/3 to-transparent rounded-2xl xs:rounded-3xl" />
          
          {/* Content */}
          <div className="relative z-10">
            
            {/* Header */}
            <div className="text-center space-y-2 xs:space-y-3 sm:space-y-4 mb-4 xs:mb-6 sm:mb-8">
              <div className="w-10 h-10 xs:w-12 xs:h-12 sm:w-14 sm:h-14 md:w-16 md:h-16 mx-auto rounded-xl xs:rounded-2xl bg-gradient-to-br from-blue-500 to-blue-600 flex items-center justify-center shadow-lg">
                <Building2 className="h-5 w-5 xs:h-6 xs:w-6 sm:h-7 sm:w-7 md:h-8 md:w-8 text-white" />
              </div>
              
              <div>
                <h1 className="text-lg xs:text-xl sm:text-2xl md:text-3xl font-bold text-gray-800 tracking-tight">
                  Welcome Back
                </h1>
                <p className="text-gray-600 text-xs xs:text-sm sm:text-base mt-1 font-medium">
                  Sign in to your RFID Hotel Management account
                </p>
              </div>
            </div>

            {/* Form */}
            <form onSubmit={handleSubmit} className="space-y-3 xs:space-y-4 sm:space-y-5">
              
              {/* Email Field */}
              <div className="space-y-1.5 xs:space-y-2">
                <Label htmlFor="email" className="text-xs xs:text-sm font-medium text-gray-700">
                  Email Address
                </Label>
                <div className="relative">
                  <Mail className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-gray-400" />
                  <Input
                    id="email"
                    type="email"
                    placeholder="Enter your email"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                    required
                    className="pl-9 xs:pl-10 h-9 xs:h-10 sm:h-11 md:h-12 glass-input dark:glass-input-dark focus:border-blue-400/50 focus:ring-2 focus:ring-blue-400/20 transition-all duration-200 text-gray-800 placeholder-gray-500 text-sm xs:text-base touch-target"
                  />
                </div>
              </div>

              {/* Password Field */}
              <div className="space-y-1.5 xs:space-y-2">
                <Label htmlFor="password" className="text-xs xs:text-sm font-medium text-gray-700">
                  Password
                </Label>
                <div className="relative">
                  <Lock className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-gray-400" />
                  <Input
                    id="password"
                    type={showPassword ? "text" : "password"}
                    placeholder="Enter your password"
                    value={password}
                    onChange={(e) => setPassword(e.target.value)}
                    required
                    className="pl-9 xs:pl-10 pr-9 xs:pr-10 h-9 xs:h-10 sm:h-11 md:h-12 glass-input dark:glass-input-dark focus:border-blue-400/50 focus:ring-2 focus:ring-blue-400/20 transition-all duration-200 text-gray-800 placeholder-gray-500 text-sm xs:text-base touch-target"
                  />
                  <button
                    type="button"
                    onClick={() => setShowPassword(!showPassword)}
                    className="absolute right-3 top-1/2 transform -translate-y-1/2 text-gray-400 hover:text-gray-600 transition-colors touch-target"
                  >
                    {showPassword ? (
                      <EyeOff className="h-4 w-4" />
                    ) : (
                      <Eye className="h-4 w-4" />
                    )}
                  </button>
                </div>
              </div>

              {/* Error Message */}
              {error && (
                <div className="text-red-500 text-xs xs:text-sm text-center bg-red-50 dark:bg-red-900/20 rounded-lg p-2 xs:p-3">
                  {error}
                </div>
              )}

              {/* Submit Button */}
              <Button
                type="submit"
                disabled={isLoading}
                className="w-full font-semibold h-9 xs:h-10 sm:h-11 md:h-12 text-sm xs:text-base shadow-lg hover:shadow-xl transition-all duration-200 transform hover:scale-[1.02] touch-target"
              >
                {isLoading ? "Signing in..." : "Sign In"}
              </Button>
            </form>

            {/* Demo Accounts */}
            <div className="mt-4 xs:mt-6 sm:mt-8 space-y-3 xs:space-y-4">
              <div className="text-center">
                <p className="text-xs xs:text-sm text-gray-500 font-medium mb-2 xs:mb-3">Quick Access - Demo Accounts</p>
              </div>
              
              <div className="space-y-2 xs:space-y-3">
                {[
                  { name: "Administrator", email: "admin@hotel.com", password: "admin123", role: "admin" },
                  { name: "Staff Member", email: "staff@hotel.com", password: "staff123", role: "staff" },
                  { name: "Guest", email: "guest@hotel.com", password: "guest123", role: "guest" }
                ].map((account, index) => (
                  <div
                    key={index}
                    className="group p-2 xs:p-3 rounded-lg bg-white/10 dark:bg-black/10 border border-white/20 dark:border-white/10 backdrop-blur-sm hover:bg-white/20 dark:hover:bg-black/20 transition-all duration-200 cursor-pointer touch-target"
                    onClick={() => {
                      setEmail(account.email)
                      setPassword(account.password)
                    }}
                  >
                    <div className="flex items-center space-x-2 xs:space-x-3">
                      <div className="w-5 h-5 xs:w-6 xs:h-6 rounded-lg bg-blue-500/20 flex items-center justify-center flex-shrink-0">
                        <Building2 className="h-3 w-3 xs:h-4 xs:w-4 text-blue-600" />
                      </div>
                      <div className="flex-1 min-w-0">
                        <p className="text-xs xs:text-sm font-medium text-gray-700 truncate">{account.name}</p>
                        <p className="text-xs text-gray-500 font-mono hidden xs:block truncate">{account.email}</p>
                      </div>
                      <div className="text-xs text-gray-500 font-mono hidden sm:block truncate">
                        {account.password}
                      </div>
                    </div>
                  </div>
                ))}
              </div>
            </div>

            {/* Device Info */}
            <div className="mt-4 xs:mt-6 text-center">
              <div className="inline-flex items-center space-x-1 xs:space-x-2 text-xs text-gray-400">
                {getDeviceIcon()}
                <span className="capitalize">{deviceType} • {orientation}</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}
