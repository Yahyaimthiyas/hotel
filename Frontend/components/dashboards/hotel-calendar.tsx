'use client'

import React, { useState, useMemo } from 'react'
import { Calendar } from '@/components/ui/calendar'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { Dialog, DialogContent, DialogHeader, DialogTitle } from '@/components/ui/dialog'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Separator } from '@/components/ui/separator'
import { useHotelData } from '@/hooks/useHotelData'
import { 
  Calendar as CalendarIcon,
  Clock,
  User,
  UserPlus,
  UserMinus,
  Shield,
  Settings,
  Bell,
  MapPin,
  Phone,
  Mail,
  CreditCard,
  Building2,
  CheckCircle,
  AlertTriangle,
  Info
} from 'lucide-react'
import { format, isSameDay, isToday, startOfMonth, endOfMonth, eachDayOfInterval } from 'date-fns'

interface HotelEvent {
  id: string
  type: 'checkin' | 'checkout' | 'maintenance' | 'security' | 'service' | 'reservation' | 'payment'
  title: string
  description: string
  date: Date
  time: string
  roomNumber?: string
  guestName?: string
  status: 'completed' | 'pending' | 'cancelled' | 'in-progress'
  priority: 'low' | 'medium' | 'high'
  assignedTo?: string
  contactInfo?: {
    phone?: string
    email?: string
  }
}

interface HotelCalendarProps {
  hotelId: string
  hotelName: string
}

// Mock data for hotel events
const generateMockEvents = (hotelId: string): HotelEvent[] => {
  const events: HotelEvent[] = []
  
  // Generate events for August 2025 (the month shown in the calendar)
  const august2025 = new Date(2025, 7, 1) // August 2025
  const startDate = startOfMonth(august2025)
  const endDate = endOfMonth(august2025)
  const daysInMonth = eachDayOfInterval({ start: startDate, end: endDate })
  
  daysInMonth.forEach((date, index) => {
    // Add some random events
    if (index % 3 === 0) {
      events.push({
        id: `checkin-${index}`,
        type: 'checkin',
        title: 'Guest Check-in',
        description: 'New guest arrival',
        date: new Date(date),
        time: '14:00',
        roomNumber: `10${index % 20 + 1}`,
        guestName: `Guest ${index + 1}`,
        status: 'completed',
        priority: 'medium',
        assignedTo: 'Front Desk',
        contactInfo: {
          phone: '+1-555-0123',
          email: `guest${index + 1}@example.com`
        }
      })
    }
    
    if (index % 4 === 0) {
      events.push({
        id: `checkout-${index}`,
        type: 'checkout',
        title: 'Guest Check-out',
        description: 'Guest departure',
        date: new Date(date),
        time: '11:00',
        roomNumber: `20${index % 15 + 1}`,
        guestName: `Guest ${index + 10}`,
        status: 'completed',
        priority: 'medium',
        assignedTo: 'Housekeeping'
      })
    }
    
    if (index % 7 === 0) {
      events.push({
        id: `maintenance-${index}`,
        type: 'maintenance',
        title: 'Room Maintenance',
        description: 'HVAC system maintenance',
        date: new Date(date),
        time: '09:00',
        roomNumber: `30${index % 10 + 1}`,
        status: 'in-progress',
        priority: 'high',
        assignedTo: 'Maintenance Team'
      })
    }
    
    if (index % 5 === 0) {
      events.push({
        id: `security-${index}`,
        type: 'security',
        title: 'Security Check',
        description: 'Routine security inspection',
        date: new Date(date),
        time: '22:00',
        status: 'completed',
        priority: 'low',
        assignedTo: 'Security Team'
      })
    }
    
    if (index % 6 === 0) {
      events.push({
        id: `service-${index}`,
        type: 'service',
        title: 'Room Service',
        description: 'Special room service request',
        date: new Date(date),
        time: '19:30',
        roomNumber: `40${index % 12 + 1}`,
        guestName: `Guest ${index + 20}`,
        status: 'pending',
        priority: 'medium',
        assignedTo: 'Room Service'
      })
    }
  })
  
  return events
}

const getEventIcon = (type: HotelEvent['type']) => {
  switch (type) {
    case 'checkin':
      return <UserPlus className="h-4 w-4 text-blue-500" />
    case 'checkout':
      return <UserMinus className="h-4 w-4 text-purple-500" />
    case 'maintenance':
      return <Settings className="h-4 w-4 text-orange-500" />
    case 'security':
      return <Shield className="h-4 w-4 text-red-500" />
    case 'service':
      return <Bell className="h-4 w-4 text-green-500" />
    case 'reservation':
      return <CalendarIcon className="h-4 w-4 text-indigo-500" />
    case 'payment':
      return <CreditCard className="h-4 w-4 text-emerald-500" />
    default:
      return <Info className="h-4 w-4 text-gray-500" />
  }
}

const getEventColor = (type: HotelEvent['type']) => {
  switch (type) {
    case 'checkin':
      return 'bg-blue-50 border-blue-200 text-blue-800 dark:bg-blue-950/20 dark:border-blue-800 dark:text-blue-300'
    case 'checkout':
      return 'bg-purple-50 border-purple-200 text-purple-800 dark:bg-purple-950/20 dark:border-purple-800 dark:text-purple-300'
    case 'maintenance':
      return 'bg-orange-50 border-orange-200 text-orange-800 dark:bg-orange-950/20 dark:border-orange-800 dark:text-orange-300'
    case 'security':
      return 'bg-red-50 border-red-200 text-red-800 dark:bg-red-950/20 dark:border-red-800 dark:text-red-300'
    case 'service':
      return 'bg-green-50 border-green-200 text-green-800 dark:bg-green-950/20 dark:border-green-800 dark:text-green-300'
    case 'reservation':
      return 'bg-indigo-50 border-indigo-200 text-indigo-800 dark:bg-indigo-950/20 dark:border-indigo-800 dark:text-indigo-300'
    case 'payment':
      return 'bg-emerald-50 border-emerald-200 text-emerald-800 dark:bg-emerald-950/20 dark:border-emerald-800 dark:text-emerald-300'
    default:
      return 'bg-gray-50 border-gray-200 text-gray-800 dark:bg-gray-950/20 dark:border-gray-800 dark:text-gray-300'
  }
}

const getEventDotColor = (type: HotelEvent['type']) => {
  switch (type) {
    case 'checkin':
      return 'bg-blue-500'
    case 'checkout':
      return 'bg-purple-500'
    case 'maintenance':
      return 'bg-orange-500'
    case 'security':
      return 'bg-red-500'
    case 'service':
      return 'bg-green-500'
    case 'reservation':
      return 'bg-indigo-500'
    case 'payment':
      return 'bg-emerald-500'
    default:
      return 'bg-gray-500'
  }
}

const getStatusColor = (status: HotelEvent['status']) => {
  switch (status) {
    case 'completed':
      return 'bg-green-100 text-green-800 dark:bg-green-900/30 dark:text-green-300'
    case 'pending':
      return 'bg-yellow-100 text-yellow-800 dark:bg-yellow-900/30 dark:text-yellow-300'
    case 'cancelled':
      return 'bg-red-100 text-red-800 dark:bg-red-900/30 dark:text-red-300'
    case 'in-progress':
      return 'bg-blue-100 text-blue-800 dark:bg-blue-900/30 dark:text-blue-300'
    default:
      return 'bg-gray-100 text-gray-800 dark:bg-gray-900/30 dark:text-gray-300'
  }
}

const getPriorityColor = (priority: HotelEvent['priority']) => {
  switch (priority) {
    case 'high':
      return 'bg-red-100 text-red-800 dark:bg-red-900/30 dark:text-red-300'
    case 'medium':
      return 'bg-yellow-100 text-yellow-800 dark:bg-yellow-900/30 dark:text-yellow-300'
    case 'low':
      return 'bg-green-100 text-green-800 dark:bg-green-900/30 dark:text-green-300'
    default:
      return 'bg-gray-100 text-gray-800 dark:bg-gray-900/30 dark:text-gray-300'
  }
}

export function HotelCalendar({ hotelId, hotelName }: HotelCalendarProps) {
  const { hotel, rooms, activities, loading, error } = useHotelData(hotelId)
  const [selectedDate, setSelectedDate] = useState<Date | undefined>(new Date())
  const [selectedEvents, setSelectedEvents] = useState<HotelEvent[]>([])
  const [isEventDialogOpen, setIsEventDialogOpen] = useState(false)
  const [selectedEvent, setSelectedEvent] = useState<HotelEvent | null>(null)

  // Generate mock events
  const events = useMemo(() => generateMockEvents(hotelId), [hotelId])

  // Get events for selected date
  const eventsForSelectedDate = useMemo(() => {
    if (!selectedDate) return []
    return events.filter(event => isSameDay(event.date, selectedDate))
  }, [events, selectedDate])

  // Get events for calendar display (grouped by date)
  const eventsByDate = useMemo(() => {
    const grouped: Record<string, HotelEvent[]> = {}
    events.forEach(event => {
      const dateKey = format(event.date, 'yyyy-MM-dd')
      if (!grouped[dateKey]) {
        grouped[dateKey] = []
      }
      grouped[dateKey].push(event)
    })
    return grouped
  }, [events])

  const handleDateSelect = (date: Date | undefined) => {
    console.log('Date selected:', date)
    setSelectedDate(date)
    if (date) {
      const dayEvents = events.filter(event => isSameDay(event.date, date))
      console.log('Events for selected date:', dayEvents)
      setSelectedEvents(dayEvents)
      // Don't auto-open dialog, let user see the events list first
    }
  }

  const handleEventClick = (event: HotelEvent) => {
    setSelectedEvent(event)
    setIsEventDialogOpen(true)
  }

  const renderDayContent = (day: Date) => {
    const dateKey = format(day, 'yyyy-MM-dd')
    const dayEvents = eventsByDate[dateKey] || []
    
    return (
      <>
        <span className="text-sm font-medium">
          {format(day, 'd')}
        </span>
        {dayEvents.length > 0 && (
          <div className="flex justify-center mt-1">
            <div className="flex flex-wrap gap-1 justify-center">
              {dayEvents.slice(0, 3).map((event, index) => (
                <div
                  key={`${event.id}-${index}`}
                  className={`w-2 h-2 rounded-full ${getEventDotColor(event.type)}`}
                  title={`${event.title} - ${event.time}`}
                />
              ))}
              {dayEvents.length > 3 && (
                <div className="w-2 h-2 rounded-full bg-gray-400" title={`+${dayEvents.length - 3} more events`} />
              )}
            </div>
          </div>
        )}
      </>
    )
  }

    return (
    <div className="h-full flex flex-col">
      <div className="grid grid-cols-1 lg:grid-cols-3 gap-4 min-h-0 flex-1">
        {/* Main Calendar */}
        <div className="lg:col-span-2">
          <Card className="h-full flex flex-col">
            <CardHeader className="pb-4 flex-shrink-0">
              <CardTitle className="flex items-center justify-between">
                <div className="flex items-center">
                  <CalendarIcon className="h-5 w-5 mr-2" />
                  Hotel Calendar
                </div>
                <div className="text-xs text-muted-foreground">
                  Click any date to view events
                </div>
              </CardTitle>
            </CardHeader>
            <CardContent className="flex-1 flex items-center justify-center pb-8 pt-0">
              <Calendar
                mode="single"
                selected={selectedDate}
                onSelect={handleDateSelect}
                className="w-full"
                components={{
                  DayButton: ({ day, ...props }) => (
                    <Button
                      {...props}
                      variant="ghost"
                      size="icon"
                      className="w-full h-full p-1 flex flex-col gap-1 hover:bg-accent/50"
                    >
                      {renderDayContent(day.date)}
                    </Button>
                  )
                }}
              />
            </CardContent>
          </Card>
        </div>

        {/* Event Legend and Quick Stats */}
        <div className="flex flex-col space-y-4 min-h-0">
          {/* Selected Date Events */}
          {selectedDate && (
            <Card className="flex-1 min-h-0 flex flex-col">
              <CardHeader className="pb-3 flex-shrink-0">
                <CardTitle className="flex items-center text-sm">
                  <Clock className="h-4 w-4 mr-2" />
                  Events for {format(selectedDate, 'MMM d')}
                </CardTitle>
              </CardHeader>
              <CardContent className="flex-1 min-h-0 p-4">
                {eventsForSelectedDate.length > 0 ? (
                  <ScrollArea className="h-full">
                    <div className="space-y-3 pr-4">
                      {eventsForSelectedDate.map((event) => (
                        <div
                          key={event.id}
                          className={`p-3 rounded-lg border cursor-pointer transition-all hover:scale-105 ${getEventColor(event.type)}`}
                          onClick={() => handleEventClick(event)}
                        >
                          <div className="flex items-start justify-between mb-2">
                            <div className="flex items-center space-x-2">
                              {getEventIcon(event.type)}
                              <span className="text-sm font-medium">{event.title}</span>
                            </div>
                            <span className="text-sm font-medium">{event.time}</span>
                          </div>
                          <p className="text-sm mb-2 text-muted-foreground">{event.description}</p>
                          <div className="flex items-center justify-between">
                            <Badge className={`text-xs ${getStatusColor(event.status)}`}>
                              {event.status}
                            </Badge>
                            {event.roomNumber && (
                              <div className="flex items-center space-x-1">
                                <Building2 className="h-3 w-3" />
                                <span className="text-xs">Room {event.roomNumber}</span>
                              </div>
                            )}
                          </div>
                        </div>
                      ))}
                    </div>
                  </ScrollArea>
                ) : (
                  <div className="text-center py-8">
                    <CalendarIcon className="h-8 w-8 text-muted-foreground mx-auto mb-3" />
                    <p className="text-muted-foreground text-sm">No events for this date</p>
                  </div>
                )}
              </CardContent>
            </Card>
          )}

          {/* Event Legend */}
          <Card className="flex-shrink-0">
            <CardHeader className="pb-2">
              <CardTitle className="text-sm">Event Types</CardTitle>
            </CardHeader>
            <CardContent className="space-y-2">
              {[
                { type: 'checkin' as const, label: 'Check-ins' },
                { type: 'checkout' as const, label: 'Check-outs' },
                { type: 'maintenance' as const, label: 'Maintenance' },
                { type: 'security' as const, label: 'Security' },
                { type: 'service' as const, label: 'Room Service' },
                { type: 'reservation' as const, label: 'Reservations' },
                { type: 'payment' as const, label: 'Payments' }
              ].map(({ type, label }) => (
                <div key={type} className="flex items-center space-x-2">
                  {getEventIcon(type)}
                  <span className="text-xs">{label}</span>
                </div>
              ))}
            </CardContent>
          </Card>
        </div>
      </div>



      {/* Event Details Dialog */}
      <Dialog open={isEventDialogOpen} onOpenChange={setIsEventDialogOpen}>
        <DialogContent className="max-w-2xl">
          <DialogHeader>
            <DialogTitle className="flex items-center">
              {selectedEvent && getEventIcon(selectedEvent.type)}
              <span className="ml-2">
                {selectedEvent?.title} - {selectedEvent && format(selectedEvent.date, 'MMM d, yyyy')}
              </span>
            </DialogTitle>
          </DialogHeader>
          
          {selectedEvent && (
            <ScrollArea className="max-h-[60vh]">
              <div className="space-y-4">
                {/* Event Details */}
                <div className="grid grid-cols-2 gap-4">
                  <div>
                    <h4 className="font-medium text-sm mb-2">Event Information</h4>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-muted-foreground">Type:</span>
                        <span className="font-medium capitalize">{selectedEvent.type}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-muted-foreground">Time:</span>
                        <span className="font-medium">{selectedEvent.time}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-muted-foreground">Status:</span>
                        <Badge className={`text-xs ${getStatusColor(selectedEvent.status)}`}>
                          {selectedEvent.status}
                        </Badge>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-muted-foreground">Priority:</span>
                        <Badge className={`text-xs ${getPriorityColor(selectedEvent.priority)}`}>
                          {selectedEvent.priority}
                        </Badge>
                      </div>
                    </div>
                  </div>
                  
                  <div>
                    <h4 className="font-medium text-sm mb-2">Assignment</h4>
                    <div className="space-y-2 text-sm">
                      <div className="flex justify-between">
                        <span className="text-muted-foreground">Assigned to:</span>
                        <span className="font-medium">{selectedEvent.assignedTo}</span>
                      </div>
                      {selectedEvent.roomNumber && (
                        <div className="flex justify-between">
                          <span className="text-muted-foreground">Room:</span>
                          <span className="font-medium">{selectedEvent.roomNumber}</span>
                        </div>
                      )}
                      {selectedEvent.guestName && (
                        <div className="flex justify-between">
                          <span className="text-muted-foreground">Guest:</span>
                          <span className="font-medium">{selectedEvent.guestName}</span>
                        </div>
                      )}
                    </div>
                  </div>
                </div>

                <Separator />

                {/* Description */}
                <div>
                  <h4 className="font-medium text-sm mb-2">Description</h4>
                  <p className="text-sm text-muted-foreground">{selectedEvent.description}</p>
                </div>

                {/* Contact Information */}
                {selectedEvent.contactInfo && (selectedEvent.contactInfo.phone || selectedEvent.contactInfo.email) && (
                  <>
                    <Separator />
                    <div>
                      <h4 className="font-medium text-sm mb-2">Contact Information</h4>
                      <div className="space-y-2">
                        {selectedEvent.contactInfo.phone && (
                          <div className="flex items-center space-x-2">
                            <Phone className="h-4 w-4 text-muted-foreground" />
                            <span className="text-sm">{selectedEvent.contactInfo.phone}</span>
                          </div>
                        )}
                        {selectedEvent.contactInfo.email && (
                          <div className="flex items-center space-x-2">
                            <Mail className="h-4 w-4 text-muted-foreground" />
                            <span className="text-sm">{selectedEvent.contactInfo.email}</span>
                          </div>
                        )}
                      </div>
                    </div>
                  </>
                )}

                {/* Actions */}
                <Separator />
                <div className="flex justify-end space-x-2">
                  <Button variant="outline" size="sm">
                    Edit Event
                  </Button>
                  <Button variant="outline" size="sm">
                    Mark Complete
                  </Button>
                  <Button size="sm">
                    View Details
                  </Button>
                </div>
              </div>
            </ScrollArea>
          )}
        </DialogContent>
      </Dialog>
    </div>
  )
} 