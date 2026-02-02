import { useState } from 'react';
import { Dashboard } from './pages/Dashboard';
import { MowerMap } from './components/MowerMap';
import { ScheduleForm } from './pages/ScheduleForm';
import { RobotProvider } from './contexts/RobotContext';
import './index.css';

type Screen =
  | { type: 'dashboard' }
  | { type: 'map'; zoneName: string }
  | { type: 'schedule' };

type Schedule = {
  zone: string;
  time: string;
};

// Initial schedules
const INITIAL_SCHEDULES: Schedule[] = [
  { zone: 'Parcelle A1', time: 'Lundi 9h30' },
  { zone: 'Jardin Arrière', time: 'Mercredi 14h00' },
];

function App() {
  const [screen, setScreen] = useState<Screen>({ type: 'dashboard' });
  const [schedules, setSchedules] = useState<Schedule[]>(INITIAL_SCHEDULES);

  const navigateToMap = (zoneName: string) => {
    setScreen({ type: 'map', zoneName });
  };

  const navigateToDashboard = () => {
    setScreen({ type: 'dashboard' });
  };

  const navigateToSchedule = () => {
    setScreen({ type: 'schedule' });
  };

  const handleScheduleSubmit = (schedule: Schedule) => {
    // In a real app, this would update the backend
    console.log('New schedule:', schedule);
    setSchedules(prev => [...prev, schedule]);
    setScreen({ type: 'dashboard' });
  };

  const renderScreen = () => {
    if (screen.type === 'dashboard') {
      return (
        <Dashboard
          onZoneClick={(zone) => navigateToMap(zone)}
          onScheduleClick={navigateToSchedule}
          schedules={schedules}
        />
      );
    }

    if (screen.type === 'map') {
      return (
        <MowerMap
          zoneName={screen.zoneName}
          onBack={navigateToDashboard}
        />
      );
    }

    if (screen.type === 'schedule') {
      return (
        <ScheduleForm
          onBack={navigateToDashboard}
          onSubmit={handleScheduleSubmit}
        />
      );
    }

    return null;
  };

  return (
    <RobotProvider>
      {renderScreen()}
    </RobotProvider>
  );
}

export default App;
