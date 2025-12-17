export const sendChatMessage = async (request) => {
  const res = await fetch('http://localhost:8001/chat', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(request)
  });
  return res.json();
};