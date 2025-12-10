import React, { useState } from 'react';

export default function UrduTranslateButton() {
  const [isUrdu, setIsUrdu] = useState(false);

  const translate = () => {
    alert("Urdu Translation is currently unavailable due to API limitations. Please contact support for a proper integration.");
  };

  return (
    <button
      onClick={translate}
      style={{
        position: 'fixed',
        bottom: '20px', /* Changed from top to bottom */
        left: '20px',
        zIndex: 99999,
        background: 'linear-gradient(135deg, #667eea, #764ba2)',
        color: 'white',
        border: 'none',
        padding: '10px 20px',
        borderRadius: '50px',
        fontSize: '14px',
        fontWeight: 'bold',
        cursor: 'pointer',
        boxShadow: '0 8px 25px rgba(0,0,0,0.3)',
        fontFamily: 'Segoe UI, sans-serif'
      }}
    >
      اردو میں ترجمہ کریں
    </button>
  );
}