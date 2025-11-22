"use client"

import { useState, useEffect } from "react"
import { useRouter } from "next/navigation"
import { 
  Building2, 
  MapPin, 
  User, 
  Mail, 
  Phone, 
  ArrowLeft, 
  Save, 
  X,
  AlertCircle,
  CheckCircle
} from "lucide-react"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Avatar, AvatarFallback, AvatarImage } from "@/components/ui/avatar"
import { Separator } from "@/components/ui/separator"
import { toast } from "sonner"

interface HotelEditPageProps {
  hotel: {
    id: string
    name: string
    location: string
    manager: {
      name: string
      phone: string
      email: string
      status: string
    }
    totalRooms?: number
    activeRooms?: number
    totalCards?: number
    status: string
    occupancy?: number
    lastActivity: string
    image: string
    description: string
  }
  onBack: () => void
  onSave: (updatedHotel: any) => void
}

interface FormData {
  name: string
  location: string
  managerName: string
  email: string
  phone: string
}

export function HotelEditPage({ hotel, onBack, onSave }: HotelEditPageProps) {
  const router = useRouter()
  const [isLoading, setIsLoading] = useState(false)
  const [formData, setFormData] = useState<FormData>({
    name: hotel.name,
    location: hotel.location,
    managerName: hotel.manager.name,
    email: hotel.manager.email || "",
    phone: hotel.manager.phone || ""
  })

  const [errors, setErrors] = useState<Partial<FormData>>({})

  // Validation function
  const validateForm = (): boolean => {
    const newErrors: Partial<FormData> = {}

    if (!formData.name.trim()) {
      newErrors.name = "Hotel name is required"
    }

    if (!formData.location.trim()) {
      newErrors.location = "Location is required"
    }

    if (!formData.managerName.trim()) {
      newErrors.managerName = "Manager name is required"
    }

    if (!formData.email.trim()) {
      newErrors.email = "Email is required"
    } else if (!/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(formData.email)) {
      newErrors.email = "Please enter a valid email address"
    }

    if (!formData.phone.trim()) {
      newErrors.phone = "Phone number is required"
    } else if (!/^[\+]?[1-9][\d]{0,15}$/.test(formData.phone.replace(/\s/g, ""))) {
      newErrors.phone = "Please enter a valid phone number"
    }

    setErrors(newErrors)
    return Object.keys(newErrors).length === 0
  }

  const handleInputChange = (field: keyof FormData, value: string) => {
    setFormData(prev => ({
      ...prev,
      [field]: value
    }))
    
    // Clear error when user starts typing
    if (errors[field]) {
      setErrors(prev => ({
        ...prev,
        [field]: undefined
      }))
    }
  }

  const handleSave = async () => {
    if (!validateForm()) {
      toast.error("Please fix the errors before saving")
      return
    }

    setIsLoading(true)

    try {
      // Simulate API call
      await new Promise(resolve => setTimeout(resolve, 1000))

      const updatedHotel = {
        ...hotel,
        name: formData.name,
        location: formData.location,
        managerName: formData.managerName,
        manager: {
          ...hotel.manager,
          name: formData.managerName,
          email: formData.email,
          phone: formData.phone
        }
      }

      onSave(updatedHotel)
      toast.success("Hotel details updated successfully!")
      
      // Navigate back after a short delay
      setTimeout(() => {
        onBack()
      }, 500)

    } catch (error) {
      toast.error("Failed to update hotel details. Please try again.")
    } finally {
      setIsLoading(false)
    }
  }

  const handleCancel = () => {
    // Check if there are unsaved changes
    const hasChanges = 
      formData.name !== hotel.name ||
      formData.location !== hotel.location ||
      formData.managerName !== hotel.managerName ||
      formData.email !== (hotel.manager?.email || "") ||
      formData.phone !== (hotel.manager?.phone || "")

    if (hasChanges) {
      if (confirm("You have unsaved changes. Are you sure you want to cancel?")) {
        onBack()
      }
    } else {
      onBack()
    }
  }

  return (
    <div className="fixed inset-0 flex bg-background overflow-hidden">
      {/* Header */}
      <div className="fixed top-0 left-0 right-0 z-50 bg-background/80 backdrop-blur-sm border-b border-border">
        <div className="flex items-center justify-between p-4 lg:p-6">
          <div className="flex items-center space-x-3">
            <Button
              variant="ghost"
              size="icon"
              onClick={handleCancel}
              className="h-8 w-8"
            >
              <ArrowLeft className="h-4 w-4" />
            </Button>
            <div className="flex items-center space-x-3">
              <div className="w-8 h-8 rounded-lg bg-blue-600 flex items-center justify-center">
                <Building2 className="h-5 w-5 text-white" />
              </div>
              <div>
                <h1 className="text-lg font-bold text-gray-900 dark:text-white">Edit Hotel</h1>
                <p className="text-sm text-gray-500 dark:text-gray-400">Update hotel information</p>
              </div>
            </div>
          </div>
          
          <div className="flex items-center space-x-2">
            <Button
              variant="outline"
              onClick={handleCancel}
              disabled={isLoading}
              className="hidden sm:flex"
            >
              <X className="h-4 w-4 mr-2" />
              Cancel
            </Button>
            <Button
              onClick={handleSave}
              disabled={isLoading}
              className="bg-blue-600 hover:bg-blue-700"
            >
              {isLoading ? (
                <div className="flex items-center space-x-2">
                  <div className="w-4 h-4 border-2 border-white border-t-transparent rounded-full animate-spin"></div>
                  <span>Saving...</span>
                </div>
              ) : (
                <>
                  <Save className="h-4 w-4 mr-2" />
                  Save Changes
                </>
              )}
            </Button>
          </div>
        </div>
      </div>

      {/* Main Content */}
      <div className="flex-1 flex flex-col pt-20 lg:pt-24">
        <div className="flex-1 overflow-y-auto p-4 lg:p-6">
          <div className="max-w-4xl mx-auto space-y-6">
            
            {/* Current Hotel Info */}
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center space-x-2">
                  <Building2 className="h-5 w-5 text-blue-600" />
                  <span>Current Hotel Information</span>
                </CardTitle>
                <CardDescription>
                  Review the current hotel details before making changes
                </CardDescription>
              </CardHeader>
              <CardContent>
                <div className="flex items-start space-x-4">
                  <Avatar className="h-16 w-16">
                    <AvatarImage src={hotel.image} alt={hotel.name} />
                    <AvatarFallback className="bg-blue-600 text-white text-lg">
                      <Building2 className="h-8 w-8" />
                    </AvatarFallback>
                  </Avatar>
                  <div className="flex-1 space-y-3">
                    <div>
                      <h3 className="text-lg font-semibold text-gray-900 dark:text-white">
                        {hotel.name}
                      </h3>
                      <p className="text-sm text-gray-500 dark:text-gray-400 flex items-center mt-1">
                        <MapPin className="h-3 w-3 mr-1" />
                        {hotel.location}
                      </p>
                    </div>
                    <div className="flex items-center space-x-4 text-sm">
                      <div className="flex items-center space-x-1">
                        <User className="h-3 w-3 text-blue-500" />
                        <span className="text-gray-600 dark:text-gray-400">
                          Manager: {hotel.managerName}
                        </span>
                      </div>
                      <div className="flex items-center space-x-1">
                        <Mail className="h-3 w-3 text-green-500" />
                        <span className="text-gray-600 dark:text-gray-400">
                          {hotel.manager?.email || "No email"}
                        </span>
                      </div>
                      <div className="flex items-center space-x-1">
                        <Phone className="h-3 w-3 text-purple-500" />
                        <span className="text-gray-600 dark:text-gray-400">
                          {hotel.manager?.phone || "No phone"}
                        </span>
                      </div>
                    </div>
                    <div className="flex items-center space-x-2">
                      <Badge variant={hotel.status === "active" ? "success" : "secondary"}>
                        {hotel.status === "active" ? "Operational" : "Inactive"}
                      </Badge>
                      <span className="text-sm text-gray-500 dark:text-gray-400">
                        {hotel.totalRooms} rooms • {hotel.activeRooms} active
                      </span>
                    </div>
                  </div>
                </div>
              </CardContent>
            </Card>

            <Separator />

            {/* Edit Form */}
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center space-x-2">
                  <CheckCircle className="h-5 w-5 text-green-600" />
                  <span>Edit Hotel Details</span>
                </CardTitle>
                <CardDescription>
                  Update the hotel information below. All changes will be reflected throughout the system.
                </CardDescription>
              </CardHeader>
              <CardContent className="space-y-6">
                
                {/* Hotel Name */}
                <div className="space-y-2">
                  <Label htmlFor="name" className="text-sm font-medium">
                    Hotel Name *
                  </Label>
                  <div className="relative">
                    <Building2 className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-gray-400" />
                    <Input
                      id="name"
                      value={formData.name}
                      onChange={(e) => handleInputChange("name", e.target.value)}
                      className={`pl-10 ${errors.name ? "border-red-500 focus:border-red-500" : ""}`}
                      placeholder="Enter hotel name"
                    />
                  </div>
                  {errors.name && (
                    <div className="flex items-center space-x-1 text-sm text-red-600">
                      <AlertCircle className="h-3 w-3" />
                      <span>{errors.name}</span>
                    </div>
                  )}
                </div>

                {/* Location */}
                <div className="space-y-2">
                  <Label htmlFor="location" className="text-sm font-medium">
                    Location *
                  </Label>
                  <div className="relative">
                    <MapPin className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-gray-400" />
                    <Input
                      id="location"
                      value={formData.location}
                      onChange={(e) => handleInputChange("location", e.target.value)}
                      className={`pl-10 ${errors.location ? "border-red-500 focus:border-red-500" : ""}`}
                      placeholder="Enter location (e.g., Salem, Tamil Nadu)"
                    />
                  </div>
                  {errors.location && (
                    <div className="flex items-center space-x-1 text-sm text-red-600">
                      <AlertCircle className="h-3 w-3" />
                      <span>{errors.location}</span>
                    </div>
                  )}
                </div>

                {/* Manager Name */}
                <div className="space-y-2">
                  <Label htmlFor="managerName" className="text-sm font-medium">
                    Manager Name *
                  </Label>
                  <div className="relative">
                    <User className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-gray-400" />
                    <Input
                      id="managerName"
                      value={formData.managerName}
                      onChange={(e) => handleInputChange("managerName", e.target.value)}
                      className={`pl-10 ${errors.managerName ? "border-red-500 focus:border-red-500" : ""}`}
                      placeholder="Enter manager name"
                    />
                  </div>
                  {errors.managerName && (
                    <div className="flex items-center space-x-1 text-sm text-red-600">
                      <AlertCircle className="h-3 w-3" />
                      <span>{errors.managerName}</span>
                    </div>
                  )}
                </div>

                {/* Email */}
                <div className="space-y-2">
                  <Label htmlFor="email" className="text-sm font-medium">
                    Email Address *
                  </Label>
                  <div className="relative">
                    <Mail className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-gray-400" />
                    <Input
                      id="email"
                      type="email"
                      value={formData.email}
                      onChange={(e) => handleInputChange("email", e.target.value)}
                      className={`pl-10 ${errors.email ? "border-red-500 focus:border-red-500" : ""}`}
                      placeholder="Enter email address"
                    />
                  </div>
                  {errors.email && (
                    <div className="flex items-center space-x-1 text-sm text-red-600">
                      <AlertCircle className="h-3 w-3" />
                      <span>{errors.email}</span>
                    </div>
                  )}
                </div>

                {/* Phone */}
                <div className="space-y-2">
                  <Label htmlFor="phone" className="text-sm font-medium">
                    Contact Number *
                  </Label>
                  <div className="relative">
                    <Phone className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-gray-400" />
                    <Input
                      id="phone"
                      value={formData.phone}
                      onChange={(e) => handleInputChange("phone", e.target.value)}
                      className={`pl-10 ${errors.phone ? "border-red-500 focus:border-red-500" : ""}`}
                      placeholder="Enter contact number"
                    />
                  </div>
                  {errors.phone && (
                    <div className="flex items-center space-x-1 text-sm text-red-600">
                      <AlertCircle className="h-3 w-3" />
                      <span>{errors.phone}</span>
                    </div>
                  )}
                </div>

              </CardContent>
            </Card>

            {/* Action Buttons */}
            <div className="flex flex-col sm:flex-row gap-3 justify-end pt-4">
              <Button
                variant="outline"
                onClick={handleCancel}
                disabled={isLoading}
                className="w-full sm:w-auto"
              >
                <X className="h-4 w-4 mr-2" />
                Cancel
              </Button>
              <Button
                onClick={handleSave}
                disabled={isLoading}
                className="w-full sm:w-auto bg-blue-600 hover:bg-blue-700"
              >
                {isLoading ? (
                  <div className="flex items-center space-x-2">
                    <div className="w-4 h-4 border-2 border-white border-t-transparent rounded-full animate-spin"></div>
                    <span>Saving Changes...</span>
                  </div>
                ) : (
                  <>
                    <Save className="h-4 w-4 mr-2" />
                    Save Changes
                  </>
                )}
              </Button>
            </div>

          </div>
        </div>
      </div>
    </div>
  )
} 