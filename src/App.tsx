import { useState } from 'react';
import { Dashboard } from './pages/Dashboard';
import { MowerMap } from './components/MowerMap';
import { ScheduleForm } from './pages/ScheduleForm';
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
  { zone: 'Parcelle B2', time: 'Mardi 14h00' },
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

  const handleScheduleSubmit = (schedule: { zone: string; day: string; time: string }) => {
    // Add new schedule to the list
    const newSchedule: Schedule = {
      zone: schedule.zone,
      time: `${schedule.day} ${schedule.time}`,
    };
    setSchedules((prev) => [...prev, newSchedule]);
    setScreen({ type: 'dashboard' });
  };

  if (screen.type === 'map') {
    return <MowerMap zoneName={screen.zoneName} onBack={navigateToDashboard} />;
  }

  if (screen.type === 'schedule') {
    return <ScheduleForm onBack={navigateToDashboard} onSubmit={handleScheduleSubmit} />;
  }

  return (
    <Dashboard 
      schedules={schedules}
      onZoneClick={navigateToMap} 
      onScheduleClick={navigateToSchedule} 
    />
  );
}

export default App;
