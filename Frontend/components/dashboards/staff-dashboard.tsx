"use client"

import { useState } from "react"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Textarea } from "@/components/ui/textarea"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import { useHotelData } from "@/hooks/useHotelData"
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from "@/components/ui/dialog"
import {
  AlertDialog,
  AlertDialogAction,
  AlertDialogCancel,
  AlertDialogContent,
  AlertDialogDescription,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogTitle,
  AlertDialogTrigger,
} from "@/components/ui/alert-dialog"
import { Table, TableBody, TableCell, TableHead, TableHeader, TableRow } from "@/components/ui/table"
import {
  CreditCard,
  Users,
  Search,
  Plus,
  Clock,
  CheckCircle,
  AlertTriangle,
  Battery,
  BatteryLow,
  MapPin,
  Phone,
  Mail,
  Calendar,
  Download,
  Edit,
  Trash2,
  Check,
  X,
  RefreshCw,
  UserCheck,
  Key,
  AlertCircle,
} from "lucide-react"


// Mock data for comprehensive staff functionality
const mockKeyCards = [
  {
    id: "KC001",
    guestName: "John Smith",
    roomNumber: "101",
    status: "active",
    batteryLevel: 85,
    issuedDate: "2024-01-14T10:00:00Z",
    expiryDate: "2024-01-17T12:00:00Z",
    issuedBy: "Sarah Staff",
    accessCount: 12,
    lastAccess: "2024-01-15T14:30:00Z",
    guestPhone: "+1-555-0123",
    guestEmail: "john.smith@email.com",
    checkInDate: "2024-01-14T15:00:00Z",
    checkOutDate: "2024-01-17T11:00:00Z",
  },
  {
    id: "KC002",
    guestName: "Sarah Johnson",
    roomNumber: "205",
    status: "active",
    batteryLevel: 45,
    issuedDate: "2024-01-15T09:00:00Z",
    expiryDate: "2024-01-18T11:00:00Z",
    issuedBy: "Sarah Staff",
    accessCount: 8,
    lastAccess: "2024-01-15T16:45:00Z",
    guestPhone: "+1-555-0124",
    guestEmail: "sarah.johnson@email.com",
    checkInDate: "2024-01-15T14:00:00Z",
    checkOutDate: "2024-01-18T10:00:00Z",
  },
  {
    id: "KC003",
    guestName: "Mike Wilson",
    roomNumber: "312",
    status: "expired",
    batteryLevel: 15,
    issuedDate: "2024-01-12T15:30:00Z",
    expiryDate: "2024-01-15T12:00:00Z",
    issuedBy: "John Staff",
    accessCount: 25,
    lastAccess: "2024-01-14T22:15:00Z",
    guestPhone: "+1-555-0125",
    guestEmail: "mike.wilson@email.com",
    checkInDate: "2024-01-12T16:00:00Z",
    checkOutDate: "2024-01-15T11:00:00Z",
  },
  {
    id: "KC004",
    guestName: "Emma Davis",
    roomNumber: "408",
    status: "inactive",
    batteryLevel: 92,
    issuedDate: "2024-01-13T14:00:00Z",
    expiryDate: "2024-01-16T12:00:00Z",
    issuedBy: "Sarah Staff",
    accessCount: 3,
    lastAccess: "2024-01-13T08:30:00Z",
    guestPhone: "+1-555-0126",
    guestEmail: "emma.davis@email.com",
    checkInDate: "2024-01-13T15:00:00Z",
    checkOutDate: "2024-01-16T11:00:00Z",
  },
]

const mockTasks = [
  {
    id: "1",
    title: "Replace low battery card for Room 312",
    description: "Guest reported card not working properly. Battery level at 15%.",
    priority: "high",
    status: "pending",
    assignedTo: "Sarah Staff",
    createdAt: "2024-01-15T13:45:00Z",
    dueDate: "2024-01-15T16:00:00Z",
    relatedCard: "KC003",
    category: "maintenance",
  },
  {
    id: "2",
    title: "Issue new card for Room 205",
    description: "Guest requested additional card for family member.",
    priority: "medium",
    status: "in-progress",
    assignedTo: "Sarah Staff",
    createdAt: "2024-01-15T12:30:00Z",
    dueDate: "2024-01-15T15:00:00Z",
    relatedCard: null,
    category: "issuance",
  },
  {
    id: "3",
    title: "Deactivate expired card KC003",
    description: "Card has expired and needs to be deactivated in the system.",
    priority: "low",
    status: "completed",
    assignedTo: "Sarah Staff",
    createdAt: "2024-01-15T10:00:00Z",
    dueDate: "2024-01-15T12:00:00Z",
    relatedCard: "KC003",
    category: "deactivation",
    completedAt: "2024-01-15T11:30:00Z",
  },
  {
    id: "4",
    title: "Guest check-in assistance for Room 101",
    description: "VIP guest arriving at 3 PM, ensure smooth check-in process.",
    priority: "high",
    status: "pending",
    assignedTo: "Sarah Staff",
    createdAt: "2024-01-15T09:00:00Z",
    dueDate: "2024-01-15T15:00:00Z",
    relatedCard: "KC001",
    category: "guest-service",
  },
]

const mockGuests = [
  {
    id: "G001",
    name: "John Smith",
    email: "john.smith@email.com",
    phone: "+1-555-0123",
    roomNumber: "101",
    checkInDate: "2024-01-14T15:00:00Z",
    checkOutDate: "2024-01-17T11:00:00Z",
    status: "checked-in",
    cardId: "KC001",
    guestType: "VIP",
    specialRequests: "Late checkout requested",
    emergencyContact: "Jane Smith - +1-555-0199",
  },
  {
    id: "G002",
    name: "Sarah Johnson",
    email: "sarah.johnson@email.com",
    phone: "+1-555-0124",
    roomNumber: "205",
    checkInDate: "2024-01-15T14:00:00Z",
    checkOutDate: "2024-01-18T10:00:00Z",
    status: "checked-in",
    cardId: "KC002",
    guestType: "Regular",
    specialRequests: "Extra towels",
    emergencyContact: "Bob Johnson - +1-555-0198",
  },
  {
    id: "G003",
    name: "Mike Wilson",
    email: "mike.wilson@email.com",
    phone: "+1-555-0125",
    roomNumber: "312",
    checkInDate: "2024-01-12T16:00:00Z",
    checkOutDate: "2024-01-15T11:00:00Z",
    status: "checked-out",
    cardId: "KC003",
    guestType: "Business",
    specialRequests: "Early breakfast",
    emergencyContact: "Lisa Wilson - +1-555-0197",
  },
]

interface StaffDashboardProps {
  hotelId?: string
}

export function StaffDashboard({ hotelId = "1" }: StaffDashboardProps) {
  const { hotel, rooms, activities, loading, error } = useHotelData(hotelId)
  const [selectedCard, setSelectedCard] = useState(null)
  const [selectedTask, setSelectedTask] = useState(null)
  const [selectedGuest, setSelectedGuest] = useState(null)
  const [isIssueCardOpen, setIsIssueCardOpen] = useState(false)
  const [isCheckInOpen, setIsCheckInOpen] = useState(false)
  const [isTaskDetailOpen, setIsTaskDetailOpen] = useState(false)
  const [cardSearchTerm, setCardSearchTerm] = useState("")
  const [cardStatusFilter, setCardStatusFilter] = useState("all")
  const [taskStatusFilter, setTaskStatusFilter] = useState("all")
  const [taskPriorityFilter, setTaskPriorityFilter] = useState("all")
  const [guestSearchTerm, setGuestSearchTerm] = useState("")
  const [guestStatusFilter, setGuestStatusFilter] = useState("all")

  const [newCard, setNewCard] = useState({
    guestName: "",
    roomNumber: "",
    guestPhone: "",
    guestEmail: "",
    checkInDate: "",
    checkOutDate: "",
    cardType: "standard",
  })

  const [newGuest, setNewGuest] = useState({
    name: "",
    email: "",
    phone: "",
    roomNumber: "",
    checkInDate: "",
    checkOutDate: "",
    guestType: "Regular",
    specialRequests: "",
    emergencyContact: "",
  })

  const todayStats = {
    cardsIssued: mockKeyCards.filter((card) => {
      const today = new Date().toDateString()
      return new Date(card.issuedDate).toDateString() === today
    }).length,
    cardsDeactivated: mockKeyCards.filter((card) => card.status === "inactive").length,
    guestsCheckedIn: mockGuests.filter((guest) => guest.status === "checked-in").length,
    pendingTasks: mockTasks.filter((task) => task.status === "pending").length,
  }

  const filteredCards = mockKeyCards.filter((card) => {
    const matchesSearch =
      card.guestName.toLowerCase().includes(cardSearchTerm.toLowerCase()) ||
      card.roomNumber.includes(cardSearchTerm) ||
      card.id.toLowerCase().includes(cardSearchTerm.toLowerCase())
    const matchesStatus = cardStatusFilter === "all" || card.status === cardStatusFilter
    return matchesSearch && matchesStatus
  })

  const filteredTasks = mockTasks.filter((task) => {
    const matchesStatus = taskStatusFilter === "all" || task.status === taskStatusFilter
    const matchesPriority = taskPriorityFilter === "all" || task.priority === taskPriorityFilter
    return matchesStatus && matchesPriority
  })

  const filteredGuests = mockGuests.filter((guest) => {
    const matchesSearch =
      guest.name.toLowerCase().includes(guestSearchTerm.toLowerCase()) ||
      guest.email.toLowerCase().includes(guestSearchTerm.toLowerCase()) ||
      guest.roomNumber.includes(guestSearchTerm)
    const matchesStatus = guestStatusFilter === "all" || guest.status === guestStatusFilter
    return matchesSearch && matchesStatus
  })

  const getPriorityColor = (priority: string) => {
    switch (priority) {
      case "high":
        return "bg-red-500"
      case "medium":
        return "bg-yellow-500"
      case "low":
        return "bg-green-500"
      default:
        return "bg-gray-500"
    }
  }

  const getStatusIcon = (status: string) => {
    switch (status) {
      case "active":
        return <CheckCircle className="h-4 w-4 text-green-500" />
      case "pending":
        return <Clock className="h-4 w-4 text-yellow-500" />
      case "expired":
        return <X className="h-4 w-4 text-red-500" />
      case "inactive":
        return <AlertTriangle className="h-4 w-4 text-orange-500" />
      default:
        return <AlertTriangle className="h-4 w-4 text-red-500" />
    }
  }

  const getTaskStatusColor = (status: string) => {
    switch (status) {
      case "completed":
        return "bg-green-100 text-green-800 border-green-200"
      case "in-progress":
        return "bg-blue-100 text-blue-800 border-blue-200"
      case "pending":
        return "bg-yellow-100 text-yellow-800 border-yellow-200"
      default:
        return "bg-gray-100 text-gray-800 border-gray-200"
    }
  }

  const handleIssueCard = () => {
    const cardId = `KC${String(mockKeyCards.length + 1).padStart(3, "0")}`
    const newCardData = {
      id: cardId,
      guestName: newCard.guestName,
      roomNumber: newCard.roomNumber,
      status: "active",
      batteryLevel: 100,
      issuedDate: new Date().toISOString(),
      expiryDate: new Date(Date.now() + 7 * 24 * 60 * 60 * 1000).toISOString(),
      issuedBy: "Sarah Staff",
      accessCount: 0,
      lastAccess: "Never",
      guestPhone: newCard.guestPhone,
      guestEmail: newCard.guestEmail,
      checkInDate: newCard.checkInDate,
      checkOutDate: newCard.checkOutDate,
    }
    mockKeyCards.push(newCardData)
    setNewCard({
      guestName: "",
      roomNumber: "",
      guestPhone: "",
      guestEmail: "",
      checkInDate: "",
      checkOutDate: "",
      cardType: "standard",
    })
    setIsIssueCardOpen(false)
  }

  const handleCheckInGuest = () => {
    const guestId = `G${String(mockGuests.length + 1).padStart(3, "0")}`
    const cardId = `KC${String(mockKeyCards.length + 1).padStart(3, "0")}`

    const guestData = {
      id: guestId,
      ...newGuest,
      status: "checked-in",
      cardId: cardId,
    }

    const cardData = {
      id: cardId,
      guestName: newGuest.name,
      roomNumber: newGuest.roomNumber,
      status: "active",
      batteryLevel: 100,
      issuedDate: new Date().toISOString(),
      expiryDate: new Date(newGuest.checkOutDate).toISOString(),
      issuedBy: "Sarah Staff",
      accessCount: 0,
      lastAccess: "Never",
      guestPhone: newGuest.phone,
      guestEmail: newGuest.email,
      checkInDate: newGuest.checkInDate,
      checkOutDate: newGuest.checkOutDate,
    }

    mockGuests.push(guestData)
    mockKeyCards.push(cardData)

    setNewGuest({
      name: "",
      email: "",
      phone: "",
      roomNumber: "",
      checkInDate: "",
      checkOutDate: "",
      guestType: "Regular",
      specialRequests: "",
      emergencyContact: "",
    })
    setIsCheckInOpen(false)
  }

  const handleDeactivateCard = (cardId: string) => {
    const card = mockKeyCards.find((c) => c.id === cardId)
    if (card) {
      card.status = "inactive"
    }
  }

  const handleCompleteTask = (taskId: string) => {
    const task = mockTasks.find((t) => t.id === taskId)
    if (task) {
      task.status = "completed"
      task.completedAt = new Date().toISOString()
    }
  }

  const exportData = (type: string) => {
    let data
    switch (type) {
      case "cards":
        data = mockKeyCards
        break
      case "tasks":
        data = mockTasks
        break
      case "guests":
        data = mockGuests
        break
      default:
        return
    }

    const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" })
    const url = URL.createObjectURL(blob)
    const a = document.createElement("a")
    a.href = url
    a.download = `${type}-export-${new Date().toISOString().split("T")[0]}.json`
    document.body.appendChild(a)
    a.click()
    document.body.removeChild(a)
    URL.revokeObjectURL(url)
  }

  return (
    <div className="space-y-4 sm:space-y-6 relative">
      {/* Welcome Section - Mobile Responsive */}
      <div className="text-center sm:text-left">
        <h1 className="text-xl sm:text-2xl md:text-3xl font-bold">Staff Dashboard</h1>
        <p className="text-sm sm:text-base text-muted-foreground mt-1">Manage key cards and assist guests</p>
      </div>

      {/* Today's Stats - Mobile Responsive */}
      <div className="grid grid-cols-2 md:grid-cols-4 gap-3 sm:gap-4">
        <Card>
          <CardContent className="p-3 sm:p-4">
            <div className="flex items-center space-x-2">
              <Plus className="h-3 w-3 sm:h-4 sm:w-4 text-green-500" />
              <div className="min-w-0 flex-1">
                <p className="text-lg sm:text-2xl font-bold truncate">{todayStats.cardsIssued}</p>
                <p className="text-xs text-muted-foreground truncate">Cards Issued</p>
              </div>
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardContent className="p-3 sm:p-4">
            <div className="flex items-center space-x-2">
              <CreditCard className="h-3 w-3 sm:h-4 sm:w-4 text-red-500" />
              <div className="min-w-0 flex-1">
                <p className="text-lg sm:text-2xl font-bold truncate">{todayStats.cardsDeactivated}</p>
                <p className="text-xs text-muted-foreground truncate">Deactivated</p>
              </div>
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardContent className="p-3 sm:p-4">
            <div className="flex items-center space-x-2">
              <Users className="h-3 w-3 sm:h-4 sm:w-4 text-blue-500" />
              <div className="min-w-0 flex-1">
                <p className="text-lg sm:text-2xl font-bold truncate">{todayStats.guestsCheckedIn}</p>
                <p className="text-xs text-muted-foreground truncate">Check-ins</p>
              </div>
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardContent className="p-3 sm:p-4">
            <div className="flex items-center space-x-2">
              <Clock className="h-3 w-3 sm:h-4 sm:w-4 text-yellow-500" />
              <div className="min-w-0 flex-1">
                <p className="text-lg sm:text-2xl font-bold truncate">{todayStats.pendingTasks}</p>
                <p className="text-xs text-muted-foreground truncate">Pending</p>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Main Content - Mobile Responsive */}
      <Tabs defaultValue="cards" className="space-y-4">
        <TabsList className="grid w-full grid-cols-3 gap-1 sm:gap-2">
          <TabsTrigger value="cards" className="text-xs sm:text-sm px-2 sm:px-3">Key Cards</TabsTrigger>
          <TabsTrigger value="tasks" className="text-xs sm:text-sm px-2 sm:px-3">Tasks</TabsTrigger>
          <TabsTrigger value="guests" className="text-xs sm:text-sm px-2 sm:px-3">Guests</TabsTrigger>
        </TabsList>

        <TabsContent value="cards" className="space-y-4">
          <Card>
            <CardHeader className="pb-3 sm:pb-4">
              <div className="flex flex-col sm:flex-row items-start sm:items-center justify-between gap-3 sm:gap-4">
                <div>
                  <CardTitle className="text-lg sm:text-xl">Key Card Management</CardTitle>
                  <CardDescription className="text-sm">Issue, manage, and monitor key cards</CardDescription>
                </div>
                <div className="flex gap-2 w-full sm:w-auto">
                  <Button onClick={() => exportData("cards")} variant="outline" size="sm" className="w-full sm:w-auto text-sm">
                    <Download className="h-3 w-3 sm:h-4 sm:w-4 mr-2" />
                    <span className="hidden sm:inline">Export</span>
                  </Button>
                  <Dialog>
                    <DialogTrigger asChild>
                      <Button size="sm" className="w-full sm:w-auto text-sm">
                        <Plus className="h-3 w-3 sm:h-4 sm:w-4 mr-2" />
                        <span className="hidden sm:inline">Issue Card</span>
                        <span className="sm:hidden">Issue</span>
                      </Button>
                    </DialogTrigger>
                    <DialogContent className="sm:max-w-[425px]">
                      <DialogHeader>
                        <DialogTitle>Issue New Key Card</DialogTitle>
                        <DialogDescription>
                          Enter guest information to issue a new key card.
                        </DialogDescription>
                      </DialogHeader>
                      <div className="grid gap-4 py-4">
                        <div className="grid grid-cols-4 items-center gap-4">
                          <Label htmlFor="guestName" className="text-right text-sm">
                            Name
                          </Label>
                          <Input
                            id="guestName"
                            value={newCard.guestName}
                            onChange={(e) => setNewCard({ ...newCard, guestName: e.target.value })}
                            className="col-span-3 text-sm"
                          />
                        </div>
                        <div className="grid grid-cols-4 items-center gap-4">
                          <Label htmlFor="roomNumber" className="text-right text-sm">
                            Room
                          </Label>
                          <Input
                            id="roomNumber"
                            value={newCard.roomNumber}
                            onChange={(e) => setNewCard({ ...newCard, roomNumber: e.target.value })}
                            className="col-span-3 text-sm"
                          />
                        </div>
                        <div className="grid grid-cols-4 items-center gap-4">
                          <Label htmlFor="guestPhone" className="text-right text-sm">
                            Phone
                          </Label>
                          <Input
                            id="guestPhone"
                            value={newCard.guestPhone}
                            onChange={(e) => setNewCard({ ...newCard, guestPhone: e.target.value })}
                            className="col-span-3 text-sm"
                          />
                        </div>
                        <div className="grid grid-cols-4 items-center gap-4">
                          <Label htmlFor="guestEmail" className="text-right text-sm">
                            Email
                          </Label>
                          <Input
                            id="guestEmail"
                            value={newCard.guestEmail}
                            onChange={(e) => setNewCard({ ...newCard, guestEmail: e.target.value })}
                            className="col-span-3 text-sm"
                          />
                        </div>
                      </div>
                      <DialogFooter>
                        <Button type="submit" onClick={handleIssueCard} className="text-sm">
                          Issue Card
                        </Button>
                      </DialogFooter>
                    </DialogContent>
                  </Dialog>
                </div>
              </div>
            </CardHeader>
            <CardContent className="space-y-4">
              <div className="flex flex-col sm:flex-row gap-4">
                <div className="relative flex-1">
                  <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-muted-foreground" />
                  <Input
                    placeholder="Search cards by guest name, room, or card ID..."
                    value={cardSearchTerm}
                    onChange={(e) => setCardSearchTerm(e.target.value)}
                    className="pl-10"
                  />
                </div>
                <Select value={cardStatusFilter} onValueChange={setCardStatusFilter}>
                  <SelectTrigger className="w-full sm:w-[180px]">
                    <SelectValue placeholder="Filter by status" />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="all">All Status</SelectItem>
                    <SelectItem value="active">Active</SelectItem>
                    <SelectItem value="inactive">Inactive</SelectItem>
                    <SelectItem value="expired">Expired</SelectItem>
                  </SelectContent>
                </Select>
              </div>

              <div className="rounded-md border">
                <Table>
                  <TableHeader>
                    <TableRow>
                      <TableHead>Guest & Room</TableHead>
                      <TableHead>Card Status</TableHead>
                      <TableHead>Battery</TableHead>
                      <TableHead>Last Access</TableHead>
                      <TableHead>Actions</TableHead>
                    </TableRow>
                  </TableHeader>
                  <TableBody>
                    {filteredCards.map((card) => (
                      <TableRow key={card.id}>
                        <TableCell>
                          <div>
                            <p className="font-medium">{card.guestName}</p>
                            <p className="text-sm text-muted-foreground">
                              Room {card.roomNumber} • {card.id}
                            </p>
                          </div>
                        </TableCell>
                        <TableCell>
                          <Badge variant="outline" className="capitalize">
                            {getStatusIcon(card.status)}
                            <span className="ml-1">{card.status}</span>
                          </Badge>
                        </TableCell>
                        <TableCell>
                          <div className="flex items-center space-x-2">
                            {card.batteryLevel < 20 ? (
                              <BatteryLow className="h-4 w-4 text-red-500" />
                            ) : (
                              <Battery className="h-4 w-4" />
                            )}
                            <span className="text-sm">{card.batteryLevel}%</span>
                          </div>
                          {card.batteryLevel < 20 && <p className="text-xs text-red-600 mt-1">Needs replacement</p>}
                        </TableCell>
                        <TableCell>
                          {card.lastAccess ? (
                            <div>
                              <p className="text-sm">{new Date(card.lastAccess).toLocaleDateString()}</p>
                              <p className="text-xs text-muted-foreground">
                                {new Date(card.lastAccess).toLocaleTimeString()}
                              </p>
                            </div>
                          ) : (
                            <span className="text-muted-foreground text-sm">Never used</span>
                          )}
                        </TableCell>
                        <TableCell>
                          <div className="flex items-center space-x-2">
                            <Button variant="ghost" size="sm">
                              <Edit className="h-4 w-4" />
                            </Button>
                            <Button variant="ghost" size="sm">
                              <RefreshCw className="h-4 w-4" />
                            </Button>
                            <AlertDialog>
                              <AlertDialogTrigger asChild>
                                <Button variant="ghost" size="sm">
                                  <Trash2 className="h-4 w-4" />
                                </Button>
                              </AlertDialogTrigger>
                              <AlertDialogContent>
                                <AlertDialogHeader>
                                  <AlertDialogTitle>Deactivate Card</AlertDialogTitle>
                                  <AlertDialogDescription>
                                    Are you sure you want to deactivate card {card.id} for {card.guestName}?
                                  </AlertDialogDescription>
                                </AlertDialogHeader>
                                <AlertDialogFooter>
                                  <AlertDialogCancel>Cancel</AlertDialogCancel>
                                  <AlertDialogAction onClick={() => handleDeactivateCard(card.id)}>
                                    Deactivate
                                  </AlertDialogAction>
                                </AlertDialogFooter>
                              </AlertDialogContent>
                            </AlertDialog>
                          </div>
                        </TableCell>
                      </TableRow>
                    ))}
                  </TableBody>
                </Table>
              </div>
            </CardContent>
          </Card>
        </TabsContent>

        <TabsContent value="tasks" className="space-y-4">
          <Card>
            <CardHeader>
              <div className="flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4">
                <div>
                  <CardTitle>Task Management</CardTitle>
                  <CardDescription>Track and manage your assigned tasks</CardDescription>
                </div>
                <div className="flex gap-2">
                  <Select value={taskStatusFilter} onValueChange={setTaskStatusFilter}>
                    <SelectTrigger className="w-[120px]">
                      <SelectValue placeholder="Status" />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="all">All</SelectItem>
                      <SelectItem value="pending">Pending</SelectItem>
                      <SelectItem value="in-progress">In Progress</SelectItem>
                      <SelectItem value="completed">Completed</SelectItem>
                    </SelectContent>
                  </Select>
                  <Select value={taskPriorityFilter} onValueChange={setTaskPriorityFilter}>
                    <SelectTrigger className="w-[120px]">
                      <SelectValue placeholder="Priority" />
                    </SelectTrigger>
                    <SelectContent>
                      <SelectItem value="all">All</SelectItem>
                      <SelectItem value="high">High</SelectItem>
                      <SelectItem value="medium">Medium</SelectItem>
                      <SelectItem value="low">Low</SelectItem>
                    </SelectContent>
                  </Select>
                  <Button onClick={() => exportData("tasks")} variant="outline" size="sm">
                    <Download className="h-4 w-4 mr-2" />
                    Export
                  </Button>
                </div>
              </div>
            </CardHeader>
            <CardContent>
              <div className="space-y-4">
                {filteredTasks.map((task) => (
                  <Card key={task.id} className="cursor-pointer hover:shadow-md transition-shadow">
                    <CardContent className="p-4">
                      <div className="flex items-start justify-between mb-3">
                        <div className="flex items-center space-x-3">
                          <div className={`w-3 h-3 rounded-full ${getPriorityColor(task.priority)}`}></div>
                          <div>
                            <h3 className="font-medium">{task.title}</h3>
                            <p className="text-sm text-muted-foreground">{task.description}</p>
                          </div>
                        </div>
                        <Badge variant="outline" className={getTaskStatusColor(task.status)}>
                          {task.status.replace("-", " ")}
                        </Badge>
                      </div>

                      <div className="flex items-center justify-between text-sm text-muted-foreground mb-3">
                        <div className="flex items-center space-x-4">
                          <span className="flex items-center">
                            <Calendar className="h-3 w-3 mr-1" />
                            Due: {new Date(task.dueDate).toLocaleDateString()}
                          </span>
                          <span className="capitalize">Priority: {task.priority}</span>
                          <span className="capitalize">Category: {task.category}</span>
                        </div>
                      </div>

                      {task.relatedCard && (
                        <div className="mb-3">
                          <Badge variant="outline" className="text-xs">
                            <Key className="h-3 w-3 mr-1" />
                            Related Card: {task.relatedCard}
                          </Badge>
                        </div>
                      )}

                      <div className="flex items-center justify-between">
                        <span className="text-xs text-muted-foreground">
                          Created: {new Date(task.createdAt).toLocaleDateString()}
                          {task.completedAt && ` • Completed: ${new Date(task.completedAt).toLocaleDateString()}`}
                        </span>
                        <div className="flex space-x-2">
                          {task.status !== "completed" && (
                            <Button size="sm" onClick={() => handleCompleteTask(task.id)}>
                              <Check className="h-4 w-4 mr-1" />
                              Complete
                            </Button>
                          )}
                          <Button variant="outline" size="sm">
                            <Edit className="h-4 w-4" />
                          </Button>
                        </div>
                      </div>
                    </CardContent>
                  </Card>
                ))}
              </div>
            </CardContent>
          </Card>
        </TabsContent>

        <TabsContent value="guests" className="space-y-4">
          <Card>
            <CardHeader>
              <div className="flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4">
                <div>
                  <CardTitle>Guest Management</CardTitle>
                  <CardDescription>View and manage guest information</CardDescription>
                </div>
                <div className="flex gap-2">
                  <Button onClick={() => exportData("guests")} variant="outline" size="sm">
                    <Download className="h-4 w-4 mr-2" />
                    Export
                  </Button>
                  <Dialog open={isCheckInOpen} onOpenChange={setIsCheckInOpen}>
                    <DialogTrigger asChild>
                      <Button size="sm">
                        <UserCheck className="h-4 w-4 mr-2" />
                        Check In Guest
                      </Button>
                    </DialogTrigger>
                    <DialogContent className="sm:max-w-[600px]">
                      <DialogHeader>
                        <DialogTitle>Guest Check-In</DialogTitle>
                        <DialogDescription>Register a new guest and issue their key card.</DialogDescription>
                      </DialogHeader>
                      <div className="grid gap-4 py-4">
                        <div className="grid grid-cols-2 gap-4">
                          <div className="grid gap-2">
                            <Label htmlFor="name">Guest Name</Label>
                            <Input
                              id="name"
                              value={newGuest.name}
                              onChange={(e) => setNewGuest((prev) => ({ ...prev, name: e.target.value }))}
                              placeholder="Enter guest name"
                            />
                          </div>
                          <div className="grid gap-2">
                            <Label htmlFor="email">Email</Label>
                            <Input
                              id="email"
                              type="email"
                              value={newGuest.email}
                              onChange={(e) => setNewGuest((prev) => ({ ...prev, email: e.target.value }))}
                              placeholder="guest@email.com"
                            />
                          </div>
                        </div>
                        <div className="grid grid-cols-2 gap-4">
                          <div className="grid gap-2">
                            <Label htmlFor="phone">Phone</Label>
                            <Input
                              id="phone"
                              value={newGuest.phone}
                              onChange={(e) => setNewGuest((prev) => ({ ...prev, phone: e.target.value }))}
                              placeholder="+1-555-0123"
                            />
                          </div>
                          <div className="grid gap-2">
                            <Label htmlFor="roomNumber">Room Number</Label>
                            <Input
                              id="roomNumber"
                              value={newGuest.roomNumber}
                              onChange={(e) => setNewGuest((prev) => ({ ...prev, roomNumber: e.target.value }))}
                              placeholder="e.g., 101"
                            />
                          </div>
                        </div>
                        <div className="grid grid-cols-2 gap-4">
                          <div className="grid gap-2">
                            <Label htmlFor="checkInDate">Check-in Date</Label>
                            <Input
                              id="checkInDate"
                              type="datetime-local"
                              value={newGuest.checkInDate}
                              onChange={(e) => setNewGuest((prev) => ({ ...prev, checkInDate: e.target.value }))}
                            />
                          </div>
                          <div className="grid gap-2">
                            <Label htmlFor="checkOutDate">Check-out Date</Label>
                            <Input
                              id="checkOutDate"
                              type="datetime-local"
                              value={newGuest.checkOutDate}
                              onChange={(e) => setNewGuest((prev) => ({ ...prev, checkOutDate: e.target.value }))}
                            />
                          </div>
                        </div>
                        <div className="grid gap-2">
                          <Label htmlFor="guestType">Guest Type</Label>
                          <Select
                            value={newGuest.guestType}
                            onValueChange={(value) => setNewGuest((prev) => ({ ...prev, guestType: value }))}
                          >
                            <SelectTrigger>
                              <SelectValue placeholder="Select guest type" />
                            </SelectTrigger>
                            <SelectContent>
                              <SelectItem value="Regular">Regular</SelectItem>
                              <SelectItem value="VIP">VIP</SelectItem>
                              <SelectItem value="Business">Business</SelectItem>
                            </SelectContent>
                          </Select>
                        </div>
                        <div className="grid gap-2">
                          <Label htmlFor="emergencyContact">Emergency Contact</Label>
                          <Input
                            id="emergencyContact"
                            value={newGuest.emergencyContact}
                            onChange={(e) => setNewGuest((prev) => ({ ...prev, emergencyContact: e.target.value }))}
                            placeholder="Name - Phone Number"
                          />
                        </div>
                        <div className="grid gap-2">
                          <Label htmlFor="specialRequests">Special Requests</Label>
                          <Textarea
                            id="specialRequests"
                            value={newGuest.specialRequests}
                            onChange={(e) => setNewGuest((prev) => ({ ...prev, specialRequests: e.target.value }))}
                            placeholder="Any special requests or notes..."
                            rows={3}
                          />
                        </div>
                      </div>
                      <DialogFooter>
                        <Button type="submit" onClick={handleCheckInGuest}>
                          Check In & Issue Card
                        </Button>
                      </DialogFooter>
                    </DialogContent>
                  </Dialog>
                </div>
              </div>
            </CardHeader>
            <CardContent className="space-y-4">
              <div className="flex flex-col sm:flex-row gap-4">
                <div className="relative flex-1">
                  <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-muted-foreground" />
                  <Input
                    placeholder="Search guests by name, email, or room..."
                    value={guestSearchTerm}
                    onChange={(e) => setGuestSearchTerm(e.target.value)}
                    className="pl-10"
                  />
                </div>
                <Select value={guestStatusFilter} onValueChange={setGuestStatusFilter}>
                  <SelectTrigger className="w-full sm:w-[180px]">
                    <SelectValue placeholder="Filter by status" />
                  </SelectTrigger>
                  <SelectContent>
                    <SelectItem value="all">All Status</SelectItem>
                    <SelectItem value="checked-in">Checked In</SelectItem>
                    <SelectItem value="checked-out">Checked Out</SelectItem>
                  </SelectContent>
                </Select>
              </div>

              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                {filteredGuests.map((guest) => (
                  <Card key={guest.id} className="cursor-pointer hover:shadow-md transition-shadow">
                    <CardContent className="p-4">
                      <div className="flex items-center justify-between mb-3">
                        <Badge variant={guest.status === "checked-in" ? "default" : "secondary"}>
                          {guest.status.replace("-", " ")}
                        </Badge>
                        <Badge variant="outline" className="text-xs">
                          {guest.guestType}
                        </Badge>
                      </div>

                      <div className="space-y-2">
                        <h3 className="font-semibold">{guest.name}</h3>
                        <div className="space-y-1 text-sm text-muted-foreground">
                          <div className="flex items-center">
                            <MapPin className="h-3 w-3 mr-1" />
                            Room {guest.roomNumber}
                          </div>
                          <div className="flex items-center">
                            <Mail className="h-3 w-3 mr-1" />
                            {guest.email}
                          </div>
                          <div className="flex items-center">
                            <Phone className="h-3 w-3 mr-1" />
                            {guest.phone}
                          </div>
                          <div className="flex items-center">
                            <Key className="h-3 w-3 mr-1" />
                            Card: {guest.cardId}
                          </div>
                        </div>

                        <div className="pt-2 border-t">
                          <div className="flex justify-between text-xs text-muted-foreground">
                            <span>Check-in: {new Date(guest.checkInDate).toLocaleDateString()}</span>
                            <span>Check-out: {new Date(guest.checkOutDate).toLocaleDateString()}</span>
                          </div>
                        </div>

                        {guest.specialRequests && (
                          <div className="pt-2">
                            <p className="text-xs text-muted-foreground">
                              <AlertCircle className="h-3 w-3 inline mr-1" />
                              {guest.specialRequests}
                            </p>
                          </div>
                        )}
                      </div>
                    </CardContent>
                  </Card>
                ))}
              </div>
            </CardContent>
          </Card>
        </TabsContent>
      </Tabs>
    </div>
  )
}
