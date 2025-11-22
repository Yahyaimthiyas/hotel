"use client"

import type React from "react"

import { useState } from "react"
import { useRouter } from "next/navigation"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card"
import { Alert, AlertDescription } from "@/components/ui/alert"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { Progress } from "@/components/ui/progress"
import { Badge } from "@/components/ui/badge"
import { FrostedGlassBackground } from "@/components/ui/frosted-glass-background"
import { useAuth, validatePassword } from "@/lib/auth"
import { Eye, EyeOff, Loader2, Mail, Lock, User, Building, AlertCircle, CheckCircle, Shield, Users, CreditCard } from "lucide-react"
import Link from "next/link"

export function SignupForm() {
  const [formData, setFormData] = useState({
    name: "",
    email: "",
    password: "",
    confirmPassword: "",
    role: "" as "administrator" | "staff" | "guest" | "",
    department: "",
  })
  const [showPassword, setShowPassword] = useState(false)
  const [showConfirmPassword, setShowConfirmPassword] = useState(false)
  const [error, setError] = useState("")
  const [isLoading, setIsLoading] = useState(false)
  const [passwordValidation, setPasswordValidation] = useState({ isValid: false, errors: [] as string[] })

  const { signup } = useAuth()
  const router = useRouter()

  const handleInputChange = (field: string, value: string) => {
    setFormData((prev) => ({ ...prev, [field]: value }))

    if (field === "password") {
      setPasswordValidation(validatePassword(value))
    }
  }

  const getPasswordStrength = (password: string): number => {
    let strength = 0
    if (password.length >= 8) strength += 20
    if (/[A-Z]/.test(password)) strength += 20
    if (/[a-z]/.test(password)) strength += 20
    if (/\d/.test(password)) strength += 20
    if (/[!@#$%^&*(),.?":{}|<>]/.test(password)) strength += 20
    return strength
  }

  const getPasswordStrengthLabel = (strength: number): string => {
    if (strength < 40) return "Weak"
    if (strength < 60) return "Fair"
    if (strength < 80) return "Good"
    return "Strong"
  }

  const getPasswordStrengthColor = (strength: number): string => {
    if (strength < 40) return "bg-destructive"
    if (strength < 60) return "bg-warning"
    if (strength < 80) return "bg-primary"
    return "bg-success"
  }

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault()
    setError("")

    // Validation
    if (!formData.name || !formData.email || !formData.password || !formData.role) {
      setError("Please fill in all required fields")
      return
    }

    if (formData.password !== formData.confirmPassword) {
      setError("Passwords do not match")
      return
    }

    if (!passwordValidation.isValid) {
      setError("Please ensure your password meets all requirements")
      return
    }

    setIsLoading(true)

    try {
      const result = await signup({
        name: formData.name,
        email: formData.email,
        password: formData.password,
        role: formData.role,
        department: formData.department || undefined,
      })

      if (result.success) {
        router.push("/dashboard")
      } else {
        setError(result.error || "Signup failed")
      }
    } catch (err) {
      setError("An unexpected error occurred")
    } finally {
      setIsLoading(false)
    }
  }

  const passwordStrength = getPasswordStrength(formData.password)

  const roleOptions = [
    { value: "administrator", label: "Administrator", icon: Shield, description: "Full system access" },
    { value: "staff", label: "Staff Member", icon: Users, description: "Limited access" },
    { value: "guest", label: "Guest", icon: CreditCard, description: "Guest services" },
  ]

  return (
    <FrostedGlassBackground>
      <div className="w-full max-w-[280px] sm:max-w-[320px] md:max-w-[360px] lg:max-w-[400px] xl:max-w-[440px]">
        {/* Enhanced Glass morphism card with improved frosted effect */}
        <div className="frosted-glass dark:frosted-glass-dark shadow-2xl rounded-2xl sm:rounded-3xl p-4 sm:p-5 md:p-6 relative overflow-hidden">
          {/* Additional inner glass layer for enhanced frosted effect */}
          <div className="absolute inset-0 bg-gradient-to-br from-white/5 to-transparent rounded-2xl sm:rounded-3xl" />
          <div className="absolute inset-0 bg-gradient-to-tl from-white/3 to-transparent rounded-2xl sm:rounded-3xl" />
          
          {/* Content container */}
          <div className="relative z-10">
            {/* Header with responsive sizing */}
                          <div className="text-center space-y-2 sm:space-y-3 mb-4 sm:mb-6">
                <div className="w-10 h-10 sm:w-12 sm:h-12 md:w-14 md:h-14 mx-auto rounded-xl sm:rounded-2xl bg-gradient-to-br from-blue-500 to-blue-600 flex items-center justify-center shadow-lg">
                  <Shield className="h-5 w-5 sm:h-6 sm:w-6 md:h-7 md:w-7 text-white" />
                </div>
                <div>
                  <h1 className="text-lg sm:text-xl md:text-2xl font-bold text-gray-800 tracking-tight">
                    Create Account
                  </h1>
                  <p className="text-gray-600 text-xs sm:text-sm mt-1 font-medium">
                    Join the hotel management system
                  </p>
                </div>
              </div>

            {/* Form with responsive spacing */}
            <form onSubmit={handleSubmit} className="space-y-3 sm:space-y-4">
              {error && (
                <Alert variant="destructive" className="border-red-200/50 bg-red-50/50 dark:bg-red-950/50 text-xs">
                  <AlertCircle className="h-3 w-3" />
                  <AlertDescription>{error}</AlertDescription>
                </Alert>
              )}

              <div className="space-y-3">
                <div className="space-y-1">
                  <Label htmlFor="name" className="text-xs font-semibold text-gray-700">
                    Full Name *
                  </Label>
                  <div className="relative group">
                    <User className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-gray-400 group-focus-within:text-blue-400 transition-colors" />
                    <Input
                      id="name"
                      type="text"
                      placeholder="Enter your full name"
                      value={formData.name}
                      onChange={(e) => handleInputChange("name", e.target.value)}
                      className="pl-9 h-9 sm:h-10 glass-input dark:glass-input-dark focus:border-blue-400/50 focus:ring-2 focus:ring-blue-400/20 transition-all duration-200 text-gray-800 placeholder-gray-500 text-sm"
                      required
                      disabled={isLoading}
                    />
                  </div>
                </div>

                <div className="space-y-1">
                  <Label htmlFor="email" className="text-xs font-semibold text-white">
                    Email Address *
                  </Label>
                  <div className="relative group">
                    <Mail className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-gray-400 group-focus-within:text-blue-400 transition-colors" />
                    <Input
                      id="email"
                      type="email"
                      placeholder="Enter your email"
                      value={formData.email}
                      onChange={(e) => handleInputChange("email", e.target.value)}
                      className="pl-9 h-9 sm:h-10 glass-input dark:glass-input-dark focus:border-blue-400/50 focus:ring-2 focus:ring-blue-400/20 transition-all duration-200 text-gray-800 placeholder-gray-500 text-sm"
                      required
                      disabled={isLoading}
                    />
                  </div>
                </div>

                <div className="space-y-1">
                  <Label htmlFor="role" className="text-xs font-semibold text-gray-700">
                    Role *
                  </Label>
                  <Select value={formData.role} onValueChange={(value) => handleInputChange("role", value)}>
                    <SelectTrigger className="h-9 sm:h-10 glass-input dark:glass-input-dark focus:border-blue-400/50 focus:ring-2 focus:ring-blue-400/20 transition-all duration-200 text-gray-800 text-sm">
                      <SelectValue placeholder="Select your role" />
                    </SelectTrigger>
                    <SelectContent className="backdrop-blur-xl bg-white/90 dark:bg-black/90 border-white/20 dark:border-white/10">
                      {roleOptions.map((role) => {
                        const IconComponent = role.icon
                        return (
                          <SelectItem key={role.value} value={role.value} className="flex items-center space-x-2">
                            <IconComponent className="h-4 w-4" />
                            <span>{role.label}</span>
                          </SelectItem>
                        )
                      })}
                    </SelectContent>
                  </Select>
                </div>

                {(formData.role === "administrator" || formData.role === "staff") && (
                  <div className="space-y-1">
                                          <Label htmlFor="department" className="text-xs font-semibold text-gray-700">
                        Department
                      </Label>
                    <div className="relative group">
                      <Building className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-gray-400 group-focus-within:text-blue-400 transition-colors" />
                      <Input
                        id="department"
                        type="text"
                        placeholder="e.g., Front Desk, Housekeeping"
                        value={formData.department}
                        onChange={(e) => handleInputChange("department", e.target.value)}
                        className="pl-9 h-9 sm:h-10 glass-input dark:glass-input-dark focus:border-blue-400/50 focus:ring-2 focus:ring-blue-400/20 transition-all duration-200 text-gray-800 placeholder-gray-500 text-sm"
                        disabled={isLoading}
                      />
                    </div>
                  </div>
                )}

                <div className="space-y-1">
                  <Label htmlFor="password" className="text-xs font-semibold text-gray-700">
                    Password *
                  </Label>
                  <div className="relative group">
                    <Lock className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-gray-400 group-focus-within:text-blue-400 transition-colors" />
                    <Input
                      id="password"
                      type={showPassword ? "text" : "password"}
                      placeholder="Create a strong password"
                      value={formData.password}
                      onChange={(e) => handleInputChange("password", e.target.value)}
                      className="pl-9 pr-9 h-9 sm:h-10 glass-input dark:glass-input-dark focus:border-blue-400/50 focus:ring-2 focus:ring-blue-400/20 transition-all duration-200 text-gray-800 placeholder-gray-500 text-sm"
                      required
                      disabled={isLoading}
                    />
                    <Button
                      type="button"
                      variant="ghost"
                      size="sm"
                      className="absolute right-1 top-1/2 transform -translate-y-1/2 h-7 w-7 p-0 hover:bg-white/20 dark:hover:bg-black/20 rounded-lg transition-colors"
                      onClick={() => setShowPassword(!showPassword)}
                      disabled={isLoading}
                    >
                      {showPassword ? (
                        <EyeOff className="h-3 w-3 text-gray-400" />
                      ) : (
                        <Eye className="h-3 w-3 text-gray-400" />
                      )}
                    </Button>
                  </div>

                  {formData.password && (
                    <div className="space-y-2 p-2 sm:p-3 rounded-lg bg-white/10 dark:bg-black/10 border border-white/20 dark:border-white/10 backdrop-blur-sm">
                      <div className="flex items-center justify-between text-xs">
                        <span className="font-medium text-gray-700">Password Strength:</span>
                        <Badge 
                          variant={passwordStrength >= 80 ? "default" : passwordStrength >= 60 ? "secondary" : "destructive"}
                          className="text-xs"
                        >
                          {getPasswordStrengthLabel(passwordStrength)}
                        </Badge>
                      </div>
                      <Progress 
                        value={passwordStrength} 
                        className="h-2 bg-white/20 dark:bg-black/20" 
                      />

                      {passwordValidation.errors.length > 0 && (
                        <div className="space-y-1">
                          {passwordValidation.errors.map((error, index) => (
                            <div key={index} className="flex items-center text-xs text-red-400">
                              <AlertCircle className="h-3 w-3 mr-1" />
                              {error}
                            </div>
                          ))}
                        </div>
                      )}
                    </div>
                  )}
                </div>

                <div className="space-y-1">
                  <Label htmlFor="confirmPassword" className="text-xs font-semibold text-white">
                    Confirm Password *
                  </Label>
                  <div className="relative group">
                    <Lock className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-gray-400 group-focus-within:text-blue-400 transition-colors" />
                    <Input
                      id="confirmPassword"
                      type={showConfirmPassword ? "text" : "password"}
                      placeholder="Confirm your password"
                      value={formData.confirmPassword}
                      onChange={(e) => handleInputChange("confirmPassword", e.target.value)}
                      className="pl-9 pr-9 h-9 sm:h-10 glass-input dark:glass-input-dark focus:border-blue-400/50 focus:ring-2 focus:ring-blue-400/20 transition-all duration-200 text-gray-800 placeholder-gray-500 text-sm"
                      required
                      disabled={isLoading}
                    />
                    <Button
                      type="button"
                      variant="ghost"
                      size="sm"
                      className="absolute right-1 top-1/2 transform -translate-y-1/2 h-7 w-7 p-0 hover:bg-white/20 dark:hover:bg-black/20 rounded-lg transition-colors"
                      onClick={() => setShowConfirmPassword(!showConfirmPassword)}
                      disabled={isLoading}
                    >
                      {showConfirmPassword ? (
                        <EyeOff className="h-3 w-3 text-gray-400" />
                      ) : (
                        <Eye className="h-3 w-3 text-gray-400" />
                      )}
                    </Button>
                  </div>

                  {formData.confirmPassword && (
                    <div className="flex items-center text-xs">
                      {formData.password !== formData.confirmPassword ? (
                        <div className="flex items-center text-red-400">
                          <AlertCircle className="h-3 w-3 mr-1" />
                          Passwords do not match
                        </div>
                      ) : (
                        <div className="flex items-center text-green-400">
                          <CheckCircle className="h-3 w-3 mr-1" />
                          Passwords match
                        </div>
                      )}
                    </div>
                  )}
                </div>
              </div>

              <Button 
                type="submit" 
                variant="gradient"
                size="lg"
                className="w-full font-semibold h-9 sm:h-10 text-sm shadow-lg hover:shadow-xl transition-all duration-200 transform hover:scale-[1.02]" 
                disabled={isLoading || !passwordValidation.isValid}
              >
                {isLoading ? (
                  <>
                    <Loader2 className="mr-2 h-4 w-4 animate-spin" />
                    Creating account...
                  </>
                ) : (
                  "Create Account"
                )}
              </Button>
            </form>

            {/* Footer with responsive spacing */}
            <div className="mt-4 sm:mt-6 text-center">
              <p className="text-xs text-gray-300">
                Already have an account?{" "}
                <Link href="/login" className="text-blue-400 hover:text-blue-300 font-semibold hover:underline transition-colors duration-200">
                  Sign in
                </Link>
              </p>
            </div>
          </div>
        </div>
      </div>
    </FrostedGlassBackground>
  )
}
