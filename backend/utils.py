import re

def clean_text(text: str) -> str:
    """
    Cleans the input text by removing excessive newlines and spaces,
    and fixing common formatting issues.
    """
    # Remove multiple newlines, keep at most two
    text = re.sub(r'\n{3,}', '\n\n', text)
    # Remove multiple spaces
    text = re.sub(r' +', ' ', text)
    # Remove leading/trailing whitespace from each line
    text = '\n'.join(line.strip() for line in text.split('\n'))
    # Remove whitespace around hyphens
    text = re.sub(r'\s*-\s*', '-', text)
    return text.strip()
