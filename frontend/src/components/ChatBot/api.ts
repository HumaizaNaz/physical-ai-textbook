export const sendChatMessage = async (request) => {
  const res = await fetch('https://humaiza-rag-chatbot.hf.space/chat', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(request)
  });
  return res.json();
};