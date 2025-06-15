import os


def prompt_template(system: str, user: str) -> str:
    msg_log = [
                {"role": "system","content": system,},
                {"role": "user", "content": user},
            ]
    
    return msg_log


















