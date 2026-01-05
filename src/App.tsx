import { useState } from 'react';
import { Dashboard } from './pages/Dashboard';
import { MowerMap } from './components/MowerMap';
import './index.css';

type Screen = 
  | { type: 'dashboard' }
  | { type: 'map'; zoneName: string };

function App() {
  const [screen, setScreen] = useState<Screen>({ type: 'dashboard' });

  const navigateToMap = (zoneName: string) => {
    setScreen({ type: 'map', zoneName });
  };

  const navigateToDashboard = () => {
    setScreen({ type: 'dashboard' });
  };

  if (screen.type === 'map') {
    return <MowerMap zoneName={screen.zoneName} onBack={navigateToDashboard} />;
  }

  return <Dashboard onZoneClick={navigateToMap} />;
}

export default App;
