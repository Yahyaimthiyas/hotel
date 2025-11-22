"use client"

import { createContext, useContext, useEffect, useState } from "react"
import type { ReactNode } from "react"

export interface User {
  id: string
  email: string
  name: string
  role: "administrator" | "staff" | "guest"
  avatar?: string
  department?: string
  permissions: string[]
  lastLogin?: string
  isActive: boolean
}

interface AuthContextType {
  user: User | null
  login: (email: string, password: string) => Promise<{ success: boolean; error?: string }>
  signup: (userData: SignupData) => Promise<{ success: boolean; error?: string }>
  logout: () => void
  loading: boolean
}

interface SignupData {
  email: string
  password: string
  name: string
  role: "administrator" | "staff" | "guest"
  department?: string
}

const AuthContext = createContext<AuthContextType | undefined>(undefined)

// Mock user database
const mockUsers: User[] = [
  {
    id: "1",
    email: "admin@hotel.com",
    name: "John Admin",
    role: "administrator",
    avatar: "/placeholder.svg?height=40&width=40",
    department: "Management",
    permissions: ["manage_users", "manage_cards", "view_reports", "system_settings"],
    lastLogin: "2024-01-15T10:30:00Z",
    isActive: true,
  },
  {
    id: "2",
    email: "staff@hotel.com",
    name: "Sarah Staff",
    role: "staff",
    avatar: "/placeholder.svg?height=40&width=40",
    department: "Front Desk",
    permissions: ["manage_cards", "view_guests"],
    lastLogin: "2024-01-15T09:15:00Z",
    isActive: true,
  },
  {
    id: "3",
    email: "guest@hotel.com",
    name: "Mike Guest",
    role: "guest",
    avatar: "/placeholder.svg?height=40&width=40",
    permissions: ["view_own_card"],
    lastLogin: "2024-01-15T14:20:00Z",
    isActive: true,
  },
]

interface AuthProviderProps {
  children: ReactNode
}

export function AuthProvider({ children }: AuthProviderProps) {
  const [user, setUser] = useState<User | null>(null)
  const [loading, setLoading] = useState(true)

  useEffect(() => {
    // Check for stored auth token
    if (typeof window !== 'undefined') {
      const storedUser = localStorage.getItem("hotel_auth_user")
      if (storedUser) {
        try {
          const user = JSON.parse(storedUser)
          setUser(user)
        } catch (error) {
          localStorage.removeItem("hotel_auth_user")
        }
      }
    }
    setLoading(false)
  }, [])

  const login = async (email: string, password: string): Promise<{ success: boolean; error?: string }> => {
    setLoading(true)

    // Simulate API call delay
    await new Promise((resolve) => setTimeout(resolve, 1000))

    const foundUser = mockUsers.find((u) => u.email === email && u.isActive)

    if (!foundUser) {
      setLoading(false)
      return { success: false, error: "Invalid email or password" }
    }

    // Validate password for demo accounts
    let isValidPassword = false
    
    // Check against known demo passwords
    if (email === "admin@hotel.com" && password === "admin123") {
      isValidPassword = true
    } else if (email === "staff@hotel.com" && password === "staff123") {
      isValidPassword = true
    } else if (email === "guest@hotel.com" && password === "guest123") {
      isValidPassword = true
    }

    if (!isValidPassword) {
      setLoading(false)
      return { success: false, error: "Invalid email or password" }
    }

    // Update last login
    foundUser.lastLogin = new Date().toISOString()

    setUser(foundUser)
    if (typeof window !== 'undefined') {
      localStorage.setItem("hotel_auth_user", JSON.stringify(foundUser))
    }
    setLoading(false)

    return { success: true }
  }

  const signup = async (userData: SignupData): Promise<{ success: boolean; error?: string }> => {
    setLoading(true)

    // Simulate API call delay
    await new Promise((resolve) => setTimeout(resolve, 1000))

    // Check if user already exists
    const existingUser = mockUsers.find((u) => u.email === userData.email)
    if (existingUser) {
      setLoading(false)
      return { success: false, error: "User with this email already exists" }
    }

    // Create new user
    const newUser: User = {
      id: Date.now().toString(),
      email: userData.email,
      name: userData.name,
      role: userData.role,
      department: userData.department,
      permissions: getPermissionsByRole(userData.role),
      isActive: true,
      lastLogin: new Date().toISOString(),
    }

    mockUsers.push(newUser)
    setUser(newUser)
    if (typeof window !== 'undefined') {
      localStorage.setItem("hotel_auth_user", JSON.stringify(newUser))
    }
    setLoading(false)

    return { success: true }
  }

  const logout = () => {
    setUser(null)
    if (typeof window !== 'undefined') {
      localStorage.removeItem("hotel_auth_user")
    }
  }

  const contextValue: AuthContextType = {
    user,
    login,
    signup,
    logout,
    loading,
  }

  return <AuthContext.Provider value={contextValue}>{children}</AuthContext.Provider>
}

export function useAuth() {
  const context = useContext(AuthContext)
  if (context === undefined) {
    throw new Error("useAuth must be used within an AuthProvider")
  }
  return context
}

function getPermissionsByRole(role: string): string[] {
  switch (role) {
    case "administrator":
      return ["manage_users", "manage_cards", "view_reports", "system_settings"]
    case "staff":
      return ["manage_cards", "view_guests"]
    case "guest":
      return ["view_own_card"]
    default:
      return []
  }
}

export function validatePassword(password: string): { isValid: boolean; errors: string[] } {
  const errors: string[] = []

  if (password.length < 8) {
    errors.push("Password must be at least 8 characters long")
  }

  if (!/[A-Z]/.test(password)) {
    errors.push("Password must contain at least one uppercase letter")
  }

  if (!/[a-z]/.test(password)) {
    errors.push("Password must contain at least one lowercase letter")
  }

  if (!/\d/.test(password)) {
    errors.push("Password must contain at least one number")
  }

  if (!/[!@#$%^&*(),.?":{}|<>]/.test(password)) {
    errors.push("Password must contain at least one special character")
  }

  return {
    isValid: errors.length === 0,
    errors,
  }
}
