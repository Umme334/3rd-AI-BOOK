import React, { useState, useEffect } from 'react';
import { Container, Paper, Box, Typography, Grid, Card, CardContent, Button, Chip, Avatar } from '@mui/material';
import { useNavigate } from 'react-router-dom';

const UserDashboard: React.FC = () => {
  const [userProfile, setUserProfile] = useState<any>(null);
  const [recentTextbooks, setRecentTextbooks] = useState<any[]>([]);
  const [activeChats, setActiveChats] = useState<any[]>([]);

  const navigate = useNavigate();

  useEffect(() => {
    // Simulate fetching user data
    const mockUserProfile = {
      id: 'user-123',
      name: 'John Doe',
      email: 'john.doe@example.com',
      softwareExperience: 'intermediate',
      hardwareExperience: 'beginner',
      primaryGoal: 'Learn humanoid robotics',
      lastLogin: new Date().toISOString()
    };

    const mockTextbooks = [
      {
        id: 'textbook-1',
        title: 'Introduction to Physical AI',
        subject: 'Physical AI and Humanoid Robotics',
        difficulty: 'intermediate',
        createdAt: '2025-12-10',
        status: 'completed'
      },
      {
        id: 'textbook-2',
        title: 'ROS 2 for Humanoid Robots',
        subject: 'Robot Operating System',
        difficulty: 'advanced',
        createdAt: '2025-12-08',
        status: 'generating'
      }
    ];

    const mockChats = [
      {
        id: 'chat-1',
        textbookId: 'textbook-1',
        textbookTitle: 'Introduction to Physical AI',
        lastMessage: 'What are the key components of ROS 2?',
        lastActivity: '2025-12-10T14:30:00Z'
      }
    ];

    setUserProfile(mockUserProfile);
    setRecentTextbooks(mockTextbooks);
    setActiveChats(mockChats);
  }, []);

  const handleCreateTextbook = () => {
    navigate('/textbook-generation');
  };

  const handleViewTextbook = (id: string) => {
    navigate(`/textbook/${id}`);
  };

  const handleChatWithTextbook = (textbookId: string) => {
    navigate(`/chatbot?textbookId=${textbookId}`);
  };

  return (
    <Container maxWidth="lg" sx={{ py: 3 }}>
      <Grid container spacing={3}>
        <Grid item xs={12}>
          <Paper sx={{ p: 3 }}>
            <Box sx={{ display: 'flex', alignItems: 'center', mb: 2 }}>
              <Avatar sx={{ width: 64, height: 64, mr: 2, bgcolor: 'primary.main' }}>
                {userProfile?.name?.charAt(0) || 'U'}
              </Avatar>
              <Box>
                <Typography variant="h4">
                  Welcome, {userProfile?.name || 'User'}!
                </Typography>
                <Typography variant="body1" color="text.secondary">
                  {userProfile?.primaryGoal || 'Personalize your Physical AI learning experience'}
                </Typography>
              </Box>
            </Box>

            <Box sx={{ display: 'flex', gap: 1, mt: 2 }}>
              <Chip
                label={`Software: ${userProfile?.softwareExperience || 'N/A'}`}
                variant="outlined"
              />
              <Chip
                label={`Hardware: ${userProfile?.hardwareExperience || 'N/A'}`}
                variant="outlined"
              />
            </Box>
          </Paper>
        </Grid>

        <Grid item xs={12}>
          <Paper sx={{ p: 3 }}>
            <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
              <Typography variant="h5">Your Textbooks</Typography>
              <Button variant="contained" onClick={handleCreateTextbook}>
                Create New Textbook
              </Button>
            </Box>

            {recentTextbooks.length > 0 ? (
              <Grid container spacing={2}>
                {recentTextbooks.map((textbook) => (
                  <Grid item xs={12} sm={6} key={textbook.id}>
                    <Card>
                      <CardContent>
                        <Typography variant="h6">{textbook.title}</Typography>
                        <Typography variant="body2" color="text.secondary" gutterBottom>
                          {textbook.subject}
                        </Typography>
                        <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                          <Chip
                            label={textbook.difficulty}
                            size="small"
                            color={textbook.difficulty === 'advanced' ? 'primary' : 'default'}
                          />
                          <Chip
                            label={textbook.status}
                            size="small"
                            color={textbook.status === 'completed' ? 'success' : 'warning'}
                          />
                        </Box>
                        <Box sx={{ mt: 2 }}>
                          <Button
                            size="small"
                            onClick={() => handleViewTextbook(textbook.id)}
                            sx={{ mr: 1 }}
                          >
                            View
                          </Button>
                          <Button
                            size="small"
                            variant="outlined"
                            onClick={() => handleChatWithTextbook(textbook.id)}
                          >
                            Chat
                          </Button>
                        </Box>
                      </CardContent>
                    </Card>
                  </Grid>
                ))}
              </Grid>
            ) : (
              <Typography variant="body1" color="text.secondary" sx={{ textAlign: 'center', py: 4 }}>
                You haven't created any textbooks yet. Start by creating your first textbook!
              </Typography>
            )}
          </Paper>
        </Grid>

        <Grid item xs={12}>
          <Paper sx={{ p: 3 }}>
            <Typography variant="h5" sx={{ mb: 2 }}>Active Chats</Typography>

            {activeChats.length > 0 ? (
              <Grid container spacing={2}>
                {activeChats.map((chat) => (
                  <Grid item xs={12} sm={6} key={chat.id}>
                    <Card>
                      <CardContent>
                        <Typography variant="h6">{chat.textbookTitle}</Typography>
                        <Typography variant="body2" color="text.secondary" gutterBottom>
                          {chat.lastMessage}
                        </Typography>
                        <Button
                          variant="contained"
                          onClick={() => handleChatWithTextbook(chat.textbookId)}
                          fullWidth
                          sx={{ mt: 1 }}
                        >
                          Continue Chat
                        </Button>
                      </CardContent>
                    </Card>
                  </Grid>
                ))}
              </Grid>
            ) : (
              <Typography variant="body1" color="text.secondary" sx={{ textAlign: 'center', py: 2 }}>
                No active chat sessions. Start a new chat with one of your textbooks.
              </Typography>
            )}
          </Paper>
        </Grid>
      </Grid>
    </Container>
  );
};

export default UserDashboard;