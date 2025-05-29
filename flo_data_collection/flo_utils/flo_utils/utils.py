def pretty_print_time(seconds):
    # Calculate days, hours, minutes, and seconds
    days = seconds // 86400
    hours = (seconds % 86400) // 3600
    minutes = (seconds % 3600) // 60
    seconds = seconds % 60
    
    # Build the pretty string
    pretty_time_parts = []
    if days > 0:
        pretty_time_parts.append(f"{round(days)}d")
    if hours > 0:
        pretty_time_parts.append(f"{round(hours)}hr")
    if minutes > 0:
        pretty_time_parts.append(f"{round(minutes)} min")
    if seconds > 0 or (hours == 0 and minutes == 0 and days == 0):
        pretty_time_parts.append(f"{round(seconds)} secs")
    
    # Join the parts with spaces and return
    return ' '.join(pretty_time_parts)