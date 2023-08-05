from typing import Callable


def require_all_subs_active(func: Callable):
    def is_all_subs_active(obj) -> bool:
        is_desired_heading_valid = obj.desired_heading is not None
        obj.get_logger().debug(
            f"Is `desired_heading` subscription valid? {is_desired_heading_valid}"
        )
        return is_desired_heading_valid

    def check(obj, *args, **kwargs):
        if is_all_subs_active(obj):
            return func(obj, *args, **kwargs)
        else:
            obj.get_logger().warn(f"All subscribers must be active to invoke {func.__name__}")
            return

    return check
