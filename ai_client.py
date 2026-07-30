from abc import ABC, abstractmethod


class AIClient(ABC):
    """Common interface so VoiceModule can talk to any text-generation API."""

    @abstractmethod
    def generate(self, prompt: str) -> str:
        """Send a prompt and return the model's text reply (empty string on no output)."""


class GeminiClient(AIClient):
    def __init__(self, api_key: str, model: str = "models/gemini-flash-latest"):
        import google.generativeai as genai

        genai.configure(api_key=api_key)
        self._model = genai.GenerativeModel(model)

    def generate(self, prompt: str) -> str:
        response = self._model.generate_content(prompt)
        if response and getattr(response, "text", None):
            return response.text.strip()
        return ""


class ClaudeClient(AIClient):
    def __init__(self, api_key: str, model: str = "claude-sonnet-5"):
        import anthropic

        self._client = anthropic.Anthropic(api_key=api_key)
        self._model = model

    def generate(self, prompt: str) -> str:
        response = self._client.messages.create(
            model=self._model,
            max_tokens=1024,
            messages=[{"role": "user", "content": prompt}],
        )
        if response and response.content:
            return "".join(
                block.text for block in response.content if block.type == "text"
            ).strip()
        return ""


_PROVIDERS = {
    "gemini": GeminiClient,
    "claude": ClaudeClient,
    "anthropic": ClaudeClient,
}


def create_ai_client(provider: str, api_key: str) -> AIClient:
    key = (provider or "gemini").strip().lower()
    client_cls = _PROVIDERS.get(key)
    if not client_cls:
        raise ValueError(
            f"Unsupported AI provider: '{provider}'. Supported: {sorted(set(_PROVIDERS))}"
        )
    return client_cls(api_key)
