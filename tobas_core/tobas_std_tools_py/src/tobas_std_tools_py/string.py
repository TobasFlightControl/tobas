import re


def is_valid_email(email: str) -> bool:
    """
    Emailアドレスが有効かどうかを判定する．
    cf. https://www.geeksforgeeks.org/check-if-email-address-valid-or-not-in-python/
    """
    regex = r"\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,7}\b"
    return re.fullmatch(regex, email)


def convert_superscript(text: str):
    """
    Converts digits following a caret (^) into their superscript equivalent.
    """
    superscript_map = {
        "0": "⁰",
        "1": "¹",
        "2": "²",
        "3": "³",
        "4": "⁴",
        "5": "⁵",
        "6": "⁶",
        "7": "⁷",
        "8": "⁸",
        "9": "⁹",
    }

    # Function to replace each match
    def replace_with_superscript(match: re.Match):
        return "".join(superscript_map[char] for char in match.group(1))

    # Replace all occurrences of ^ followed by one or more digits
    return re.sub(r"\^(\d+)", replace_with_superscript, text)
