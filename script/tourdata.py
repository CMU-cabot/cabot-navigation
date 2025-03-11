from dataclasses import dataclass
from typing import List, Optional
import re

@dataclass
class Destination:
    floor: Optional[int]
    value: Optional[str]
    sharp_title: Optional[str]
    var: Optional[str] = None  # Make this field optional
    arrivalAngle: Optional[float] = None  # Make this field optional
    content: Optional[str] = None  # Add this field


@dataclass
class Message:
    type: Optional[str]
    parent: Optional[str]
    text: Optional[dict] = None  # Use a dictionary for text fields


@dataclass
class TourDestination:
    ref: Optional[str]
    ref_title: Optional[str] = None  # Add this field
    sharp_ref: Optional[str] = None  # Add this field


@dataclass
class Tour:
    tour_id: Optional[str]
    destinations: List[TourDestination]
    title: Optional[dict] = None  # Use a dictionary for text fields
    default_var: Optional[str] = None  # Add this field
    debug: Optional[bool] = None  # Add this field


@dataclass
class TourData:
    destinations: List[Destination]
    messages: List[Message]
    tours: List[Tour]


def get_required_field(data, field_name):
    if field_name not in data:
        raise ValueError(f"Missing required field: {field_name}")
    return data[field_name]


def map_data_to_class(data, cls):
    field_names = {f.name for f in cls.__dataclass_fields__.values()}
    mapped_data = {}
    for key, value in data.items():
        if key.startswith("#"):
            key = "sharp_" + key[1:]

        items = re.split('[:\-]', key)
        if len(items) > 1:
            base_key = items[0]
            lang = "".join(items[1:])
            if base_key in mapped_data:
                mapped_data[base_key][lang] = value
            else:
                mapped_data[base_key] = {lang: value}
        elif len(items) == 1 and key in field_names:
            mapped_data[key] = value
        else:
            raise ValueError(f"Unexpected field '{key}' in dataclass '{cls.__name__}'")

    return cls(**mapped_data)


def parse_tour_data(data) -> TourData:
    destinations = [
        map_data_to_class(d, Destination)
        for d in data.get("destinations", [])
    ]

    messages = [
        map_data_to_class(m, Message)
        for m in data.get("messages", [])
    ]

    tours = []
    for t in data.get("tours", []):
        tour_destinations = [
            map_data_to_class(td, TourDestination)
            for td in t.get("destinations", [])
        ]
        tour_data = map_data_to_class(t, Tour)
        tour_data.destinations = tour_destinations
        tours.append(tour_data)

    return TourData(destinations=destinations, messages=messages, tours=tours)
