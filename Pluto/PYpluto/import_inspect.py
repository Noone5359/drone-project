import inspect
import typing
from contextlib import suppress
from functools import wraps

def get_origin(type_hint):
    if is_unparameterized_special_typing(type_hint):
        return get_origin(type_hint.__args__[0])
    return type_hint


# This is a file to wrap the pluto class made in drone.py. 
def is_unparameterized_special_typing(type_hint):
    return (
        hasattr(typing, "_SpecialForm") and isinstance(type_hint, typing._SpecialForm)
        or (hasattr(type_hint, "__origin__") and type_hint.__origin__ is None)
    )

def enforce_types(target):
    #Class decorator adding type checks to all member functions
    # type_check-> wrap

    def check_types(spec, *args, **kwargs):
        parameters = dict(zip(spec.args, args))
        parameters.update(kwargs)
        
        for name, value in parameters.items():
            with suppress(KeyError):  # Assume un-annotated parameters can be any type
                type_hint = spec.annotations[name]
                
                # Handle cases with generics and special types
                origin = get_origin(type_hint)
                expected_type = origin if origin is not None else type_hint
                
                if not isinstance(value, expected_type):
                    raise TypeError(f"Unexpected type for '{name}' (expected {type_hint} but found {type(value)})")

    def decorate(func):
        spec = inspect.getfullargspec(func)

        @wraps(func) #wrapping done
        def wrapper(*args, **kwargs):
            check_types(spec, *args, **kwargs)
            return func(*args, **kwargs)

        return wrapper
    #Function decorator adding type checks

    if inspect.isclass(target):
        members = inspect.getmembers(target, predicate=inspect.isfunction)
        for name, func in members:
            setattr(target, name, decorate(func))

        return target
    else:
        return decorate(target)