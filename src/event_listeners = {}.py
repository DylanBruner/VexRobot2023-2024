class EventSystem:
    def __init__(self):
        self.event_listeners: dict[str, list[callable]] = {}

    def subscribe(self, event_name: str):
        def decorator(func: callable):
            if event_name not in self.event_listeners:
                self.event_listeners[event_name] = []
            self.event_listeners[event_name].append(func)
            return func
        return decorator
    
    def dispatch(self, event_name: str, event: object):
        if event_name in self.event_listeners:
            for event_listener in self.event_listeners[event_name]:
                event_listener(event)
    

event_system = EventSystem()

@event_system.subscribe("preInit")
def event_handler(event):
    print("handling event", event)

@event_system.subscribe("preInit")
def event_handler2(event):
    print("handling event 2", event)

event_system.dispatch("preInit", {})